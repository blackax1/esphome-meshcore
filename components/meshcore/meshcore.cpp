#include "meshcore.h"

#include <base64.hpp>

#include "esphome/core/log.h"

#include <Utils.h>
#include <helpers/AdvertDataHelpers.h>

namespace esphome {
namespace meshcore {

static const char *const TAG = "meshcore";

// Mirrors the upstream simple_repeater example. Bump if rx queues fill
// up under heavy traffic.
static constexpr int PACKET_POOL_SIZE = 16;

// Max text body for a group message, matching BaseChatMesh's MAX_TEXT_LEN
// (10 * CIPHER_BLOCK_SIZE = 160). Defined locally so we don't have to
// pull in all of BaseChatMesh.h just for one constant.
static constexpr size_t MESHCORE_MAX_TEXT_LEN = 10 * CIPHER_BLOCK_SIZE;

// "Real world" floor for accepting mesh-learned timestamps: 2024-01-01
// 00:00:00 UTC. Stops a misbehaving / freshly-booted peer from
// fast-forwarding our clock to junk values. MeshCore's own RTC bootstrap
// uses 2024-05-15 (1715770351); we go a few months earlier so our
// floor always trails the upstream baseline by a comfortable margin.
static constexpr uint32_t MESH_TIME_FLOOR = 1704067200U;

// Minimum delta in seconds between the incoming timestamp and the
// current RTC before we bother bumping. Keeps small jitter from
// constantly re-writing NVS as adverts arrive.
static constexpr uint32_t MIN_MESH_TIME_BUMP_SEC = 60;

// ---- MeshCoreComponent ----

void MeshCoreComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MeshCore hub '%s'...", this->node_name_.c_str());

  this->board_.begin();
  this->rtc_clock_.begin();

  this->radio_ = std::unique_ptr<CustomSX1262>(
      new CustomSX1262(new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY)));

  if (!this->radio_->std_init(&SPI)) {
    ESP_LOGE(TAG, "SX1262 init failed; mesh stack will stay disabled");
    this->mark_failed();
    return;
  }

  this->radio_wrapper_ = std::unique_ptr<CustomSX1262Wrapper>(
      new CustomSX1262Wrapper(*this->radio_, this->board_));

  this->packet_mgr_ = std::unique_ptr<StaticPoolPacketManager>(
      new StaticPoolPacketManager(PACKET_POOL_SIZE));

  this->mesh_ = std::unique_ptr<EsphomeMesh>(new EsphomeMesh(
      this, *this->radio_wrapper_, this->ms_clock_, this->rng_, this->rtc_clock_,
      *this->packet_mgr_, this->tables_));

  // Seed the RNG before any identity work that might use it.
  this->rng_.begin(esp_random());

  if (!this->load_or_create_identity_()) {
    ESP_LOGE(TAG, "Identity setup failed; mesh stack will stay disabled");
    this->mark_failed();
    return;
  }

  // Restore any previously mesh-learned timestamp from NVS so we don't
  // start from MeshCore's 2024 baseline on every boot. Cheap, fixed
  // 32-bit blob keyed off the node name (so two nodes flashed onto the
  // same board don't share state).
  this->mesh_time_pref_ = global_preferences->make_preference<uint32_t>(
      fnv1_hash(std::string("meshcore.mesh_time.") + this->node_name_));
  uint32_t saved_ts = 0;
  if (this->mesh_time_pref_.load(&saved_ts) && saved_ts >= MESH_TIME_FLOOR) {
    this->rtc_clock_.setCurrentTime(saved_ts);
    ESP_LOGCONFIG(TAG, "RTC restored from NVS: %u", (unsigned) saved_ts);
  }

  // Decode any channels declared in YAML now that hashing helpers are
  // available. We mirror BaseChatMesh::addChannel's logic so we don't
  // have to inherit from it.
  for (const auto &pc : this->pending_channels_) {
    if (this->channels_.size() >= 16) {  // hard ceiling; tune if needed
      ESP_LOGW(TAG, "channel '%s' dropped: too many channels configured", pc.name.c_str());
      break;
    }
    ChannelDetails ch{};
    memset(ch.channel.secret, 0, sizeof(ch.channel.secret));
    int len = decode_base64(reinterpret_cast<unsigned char *>(const_cast<char *>(pc.psk_base64.data())),
                            pc.psk_base64.size(), ch.channel.secret);
    if (len != 16 && len != 32) {
      ESP_LOGW(TAG, "channel '%s' rejected: PSK must decode to 16 or 32 bytes (got %d)",
               pc.name.c_str(), len);
      continue;
    }
    mesh::Utils::sha256(ch.channel.hash, sizeof(ch.channel.hash), ch.channel.secret, len);
    StrHelper::strncpy(ch.name, pc.name.c_str(), sizeof(ch.name));
    this->channels_.push_back(ch);
    ESP_LOGCONFIG(TAG, "  Channel '%s' loaded (%d-bit PSK)", pc.name.c_str(), len * 8);
  }
  this->pending_channels_.clear();
  this->pending_channels_.shrink_to_fit();

  this->mesh_->begin();
  this->ready_ = true;

  ESP_LOGCONFIG(TAG, "MeshCore ready, pubkey prefix = %02x%02x%02x%02x",
                this->mesh_->self_id.pub_key[0], this->mesh_->self_id.pub_key[1],
                this->mesh_->self_id.pub_key[2], this->mesh_->self_id.pub_key[3]);
}

void MeshCoreComponent::loop() {
  if (!this->ready_) {
    return;
  }
  this->mesh_->loop();
  this->rtc_clock_.tick();

  const uint32_t now = millis();
  if (now - this->last_battery_pub_ms_ > 60000U) {
    this->last_battery_pub_ms_ = now;
    this->on_battery_sample_();
  }
}

void MeshCoreComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "MeshCore:");
  ESP_LOGCONFIG(TAG, "  Node name: %s", this->node_name_.c_str());
  ESP_LOGCONFIG(TAG, "  Frequency: %.4f MHz", (float) LORA_FREQ);
  ESP_LOGCONFIG(TAG, "  Bandwidth: %.2f kHz", (float) LORA_BW);
  ESP_LOGCONFIG(TAG, "  Spreading factor: %d", (int) LORA_SF);
  ESP_LOGCONFIG(TAG, "  Coding rate: 4/%d", (int) LORA_CR);
  ESP_LOGCONFIG(TAG, "  TX power: %d dBm", (int) LORA_TX_POWER);
  ESP_LOGCONFIG(TAG, "  Pins: SCLK=%d MISO=%d MOSI=%d NSS=%d DIO1=%d RST=%d BUSY=%d",
                P_LORA_SCLK, P_LORA_MISO, P_LORA_MOSI, P_LORA_NSS,
                P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY);
  ESP_LOGCONFIG(TAG, "  Channels configured: %u", (unsigned) this->channels_.size());
  ESP_LOGCONFIG(TAG, "  Identity source: %s",
                this->static_identity_hex_.empty() ? "NVS (auto)" : "YAML private_key");
  ESP_LOGCONFIG(TAG, "  RTC time: %u (%s)",
                (unsigned) this->rtc_clock_.getCurrentTime(),
                this->rtc_clock_.getCurrentTime() >= MESH_TIME_FLOOR
                    ? "synced from mesh"
                    : "baseline (waiting for advert)");
  ESP_LOGCONFIG(TAG, "  Mesh ready: %s", YESNO(this->ready_));
}

void MeshCoreComponent::add_channel(const std::string &name, const std::string &psk_base64) {
  this->pending_channels_.push_back({name, psk_base64});
}

bool MeshCoreComponent::send_text_message(const std::string &text) {
  if (this->channels_.empty()) {
    ESP_LOGW(TAG, "send_text_message dropped: no channels configured");
    return false;
  }
  return this->send_text_message(this->channels_.front().name, text);
}

bool MeshCoreComponent::send_text_message(const std::string &channel_name, const std::string &text) {
  if (!this->ready_) {
    ESP_LOGW(TAG, "send_text_message dropped: mesh not ready");
    return false;
  }
  // Locate the configured channel by name.
  const ChannelDetails *channel = nullptr;
  for (const auto &ch : this->channels_) {
    if (channel_name == ch.name) {
      channel = &ch;
      break;
    }
  }
  if (channel == nullptr) {
    ESP_LOGW(TAG, "send_text_message: channel '%s' not configured", channel_name.c_str());
    return false;
  }

  if (text.size() > MESHCORE_MAX_TEXT_LEN) {
    ESP_LOGW(TAG, "send_text_message: trimming '%s' to %u bytes", text.c_str(),
             (unsigned) MESHCORE_MAX_TEXT_LEN);
  }

  // Wire format expected by other MeshCore nodes on PAYLOAD_TYPE_GRP_TXT:
  //   [timestamp(4) | txt_type(1) | "<sender>: <msg>\0"]
  // See BaseChatMesh::sendGroupMessage in the upstream library.
  const std::string sender = this->node_name_;
  const size_t prefix_len = sender.size() + 2;  // "name: "
  const size_t text_len = std::min(text.size(), MESHCORE_MAX_TEXT_LEN - prefix_len);
  const size_t total = 5 + prefix_len + text_len;

  uint8_t buf[5 + 10 * CIPHER_BLOCK_SIZE + 32];
  const uint32_t ts = this->rtc_clock_.getCurrentTime();
  memcpy(buf, &ts, 4);
  buf[4] = TXT_TYPE_PLAIN;
  memcpy(&buf[5], sender.data(), sender.size());
  buf[5 + sender.size()] = ':';
  buf[5 + sender.size() + 1] = ' ';
  memcpy(&buf[5 + prefix_len], text.data(), text_len);

  mesh::Packet *pkt = this->mesh_->createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, channel->channel, buf, total);
  if (pkt == nullptr) {
    ESP_LOGW(TAG, "send_text_message: packet allocation failed");
    return false;
  }
  this->mesh_->sendFlood(pkt);
  ESP_LOGD(TAG, "send_text_message flooded on '%s': %s", channel->name, text.c_str());
  return true;
}

void MeshCoreComponent::on_message_received(const std::string &payload, float rssi, float snr) {
  ESP_LOGD(TAG, "rx: '%s' rssi=%.1f snr=%.1f", payload.c_str(), rssi, snr);
  if (this->last_message_sensor_ != nullptr) {
    this->last_message_sensor_->publish_last_message(payload);
  }
  if (this->sensors_ != nullptr) {
    this->sensors_->publish_rssi(rssi);
    this->sensors_->publish_snr(snr);
  }
}

void MeshCoreComponent::on_battery_sample_() {
  if (this->sensors_ == nullptr) {
    return;
  }
  const uint16_t mv = this->board_.getBattMilliVolts();
  if (mv == 0) {
    return;  // not configured on this board
  }
  this->sensors_->publish_battery_voltage(mv / 1000.0f);
}

void MeshCoreComponent::bump_rtc_from_mesh(uint32_t mesh_timestamp) {
  if (mesh_timestamp < MESH_TIME_FLOOR) {
    return;  // not plausible "real world" time, ignore
  }
  const uint32_t now = this->rtc_clock_.getCurrentTime();
  if (mesh_timestamp <= now + MIN_MESH_TIME_BUMP_SEC) {
    return;  // already current enough
  }
  this->rtc_clock_.setCurrentTime(mesh_timestamp);
  ESP_LOGD(TAG, "rtc bumped from mesh: %u -> %u (+%u s)",
           (unsigned) now, (unsigned) mesh_timestamp,
           (unsigned) (mesh_timestamp - now));

  // Persist to NVS so we don't fall back to the hard-coded baseline on
  // the next boot. Cheap because the >60s guard keeps writes rare.
  uint32_t blob = mesh_timestamp;
  this->mesh_time_pref_.save(&blob);
  global_preferences->sync();
}

bool MeshCoreComponent::load_or_create_identity_() {
  // 1. YAML-supplied static identity beats everything else. Lets the
  //    operator pin a node's pubkey across reflashes (handy for
  //    contact lists, telemetry routing, dashboards).
  if (!this->static_identity_hex_.empty()) {
    uint8_t blob[PRV_KEY_SIZE + PUB_KEY_SIZE];
    const size_t hex_len = this->static_identity_hex_.size();
    const size_t bytes = hex_len / 2;  // 64 (prv only) or 96 (prv+pub)
    if (!mesh::Utils::fromHex(blob, bytes, this->static_identity_hex_.c_str())) {
      ESP_LOGE(TAG, "private_key hex decode failed; falling back to NVS");
    } else {
      this->mesh_->self_id.readFrom(blob, bytes);
      ESP_LOGCONFIG(TAG, "Loaded static identity from YAML private_key");
      return true;
    }
  }

  // 2. NVS cache. Persists the keypair generated on first boot so we
  //    don't look like a brand-new node to peers after every reset.
  this->identity_pref_ = global_preferences->make_preference<uint8_t[PRV_KEY_SIZE + PUB_KEY_SIZE]>(
      fnv1_hash(std::string("meshcore.identity.") + this->node_name_));

  uint8_t blob[PRV_KEY_SIZE + PUB_KEY_SIZE];
  if (this->identity_pref_.load(&blob)) {
    this->mesh_->self_id.readFrom(blob, sizeof(blob));
    ESP_LOGCONFIG(TAG, "Loaded persistent identity from preferences");
    return true;
  }

  // 3. First boot, no YAML override: generate and cache.
  ESP_LOGCONFIG(TAG, "No identity in preferences, generating a new keypair");
  this->mesh_->self_id = mesh::LocalIdentity(this->mesh_->getRNG());
  if (this->mesh_->self_id.writeTo(blob, sizeof(blob)) != sizeof(blob) ||
      !this->identity_pref_.save(&blob)) {
    ESP_LOGW(TAG, "Failed to persist new identity; will regenerate on next boot");
    // Non-fatal: we can still run, just non-persistent.
  } else {
    global_preferences->sync();
  }
  return true;
}

// ---- EsphomeMesh callbacks ----

void EsphomeMesh::onAdvertRecv(mesh::Packet *packet, const mesh::Identity &id, uint32_t timestamp,
                               const uint8_t *app_data, size_t app_data_len) {
  // app_data here is the raw advert payload (typically a node name +
  // optional location). We log it but don't push to the message
  // text_sensor: real chat traffic comes via onGroupDataRecv.
  std::string label(reinterpret_cast<const char *>(app_data), app_data_len);
  ESP_LOGD(TAG, "advert from %02x%02x%02x%02x: '%s'",
           id.pub_key[0], id.pub_key[1], id.pub_key[2], id.pub_key[3], label.c_str());
  // Bootstrap our RTC off whichever advert in the mesh has the newest
  // timestamp. Off-grid nodes converge their clock this way without
  // needing WiFi / NTP / GPS.
  this->owner_->bump_rtc_from_mesh(timestamp);
}

void EsphomeMesh::onAnonDataRecv(mesh::Packet *packet, const uint8_t *secret,
                                 const mesh::Identity &sender, uint8_t *data, size_t len) {
  std::string payload(reinterpret_cast<const char *>(data), len);
  const float rssi = this->_radio->getLastRSSI();
  const float snr = this->_radio->getLastSNR();
  this->owner_->on_message_received(payload, rssi, snr);
}

int EsphomeMesh::searchChannelsByHash(const uint8_t *hash, mesh::GroupChannel channels[],
                                      int max_matches) {
  int matched = 0;
  for (const auto &ch : this->owner_->channels()) {
    if (memcmp(ch.channel.hash, hash, PATH_HASH_SIZE) == 0) {
      if (matched < max_matches) {
        channels[matched] = ch.channel;
      }
      matched++;
    }
  }
  return matched;
}

void EsphomeMesh::onGroupDataRecv(mesh::Packet *packet, uint8_t type,
                                  const mesh::GroupChannel &channel, uint8_t *data, size_t len) {
  if (type != PAYLOAD_TYPE_GRP_TXT) {
    return;  // ignore opaque group datagrams for now
  }
  if (len < 6) {
    return;
  }
  // Wire format: [timestamp(4) | txt_type(1) | "sender: text\0"]
  // We just hand the human-readable part straight to the text sensor.
  uint32_t sender_ts;
  memcpy(&sender_ts, data, 4);
  this->owner_->bump_rtc_from_mesh(sender_ts);

  const char *text = reinterpret_cast<const char *>(&data[5]);
  std::string payload(text, strnlen(text, len - 5));
  const float rssi = this->_radio->getLastRSSI();
  const float snr = this->_radio->getLastSNR();
  this->owner_->on_message_received(payload, rssi, snr);
}

}  // namespace meshcore
}  // namespace esphome
