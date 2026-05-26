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

  // Construct the concrete radio. The Module(...) constructor takes
  // different signal pins for the two families:
  //   SX126x: NSS, DIO1, RESET, BUSY (BUSY is the SX126x-specific
  //           "modem busy" pin)
  //   SX127x: NSS, DIO0, RESET, DIO1 (DIO0 = packet-done IRQ; no
  //           BUSY pin on this family). DIO1 is optional on SX127x;
  //           pass RADIOLIB_NC when omitted so RadioLib polls.
#if defined(USE_SX1262)
  this->radio_ = std::unique_ptr<ConcreteRadio>(
      new ConcreteRadio(new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY)));
#elif defined(USE_SX1276)
  this->radio_ = std::unique_ptr<ConcreteRadio>(
      new ConcreteRadio(new Module(P_LORA_NSS, P_LORA_DIO_0, P_LORA_RESET, P_LORA_DIO_1)));
#endif

  // Rather than calling MeshCore's std_init() (which logs to Serial
  // and returns just a bool), drive RadioLib's begin() directly so
  // any failure surfaces with a numeric error code in the ESPHome log.
  // The parameters mirror upstream's CustomSX1262/SX1276::std_init
  // helpers so behaviour is the same on success.
  SPI.begin(P_LORA_SCLK, P_LORA_MISO, P_LORA_MOSI);
#if defined(USE_SX1262)
  float tcxo = SX126X_DIO3_TCXO_VOLTAGE;
  int16_t status = this->radio_->begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR,
                                       RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
                                       LORA_TX_POWER, 16, tcxo);
  if (status == RADIOLIB_ERR_SPI_CMD_FAILED || status == RADIOLIB_ERR_SPI_CMD_INVALID) {
    // Common on boards without a TCXO: retry with the XOSC (0V) path.
    ESP_LOGW(TAG, "radio init: TCXO failed (status=%d), retrying with XOSC", status);
    status = this->radio_->begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR,
                                 RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
                                 LORA_TX_POWER, 16, 0.0f);
  }
#elif defined(USE_SX1276)
  int16_t status = this->radio_->begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR,
                                       RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
                                       LORA_TX_POWER, 16);
#endif

  if (status != RADIOLIB_ERR_NONE) {
    ESP_LOGE(TAG, "radio init failed (RadioLib status=%d); mesh stack will stay disabled",
             (int) status);
    ESP_LOGE(TAG, "  RadioLib error code reference (sx126x/sx127x):");
    ESP_LOGE(TAG, "    -2:   chip not found (radio not responding to SPI)");
    ESP_LOGE(TAG, "    -8:   invalid bandwidth for this chip");
    ESP_LOGE(TAG, "    -9:   invalid spreading factor for this chip");
    ESP_LOGE(TAG, "    -10:  invalid coding rate for this chip");
    ESP_LOGE(TAG, "    -12:  invalid frequency for this chip");
    ESP_LOGE(TAG, "    -13:  invalid output power (try lowering tx_power; SX1276");
    ESP_LOGE(TAG, "          PA_BOOST tops out around +17 dBm with default config)");
    ESP_LOGE(TAG, "    -707: SPI cmd failed (TCXO mismatch on SX126x)");
    ESP_LOGE(TAG, "  Common causes: wrong SPI/DIO/RST pins; radio LDO unpowered;");
    ESP_LOGE(TAG, "  dio1_pin set on a board that doesn't wire DIO1 (try omitting).");
    this->mark_failed();
    return;
  }

  // CRC + any radio-family-specific tunings that std_init applied.
  this->radio_->setCRC(1);
#if defined(USE_SX1262)
#ifdef SX126X_DIO2_AS_RF_SWITCH
  this->radio_->setDio2AsRfSwitch(SX126X_DIO2_AS_RF_SWITCH);
#endif
#ifdef SX126X_RX_BOOSTED_GAIN
  this->radio_->setRxBoostedGainMode(SX126X_RX_BOOSTED_GAIN);
#endif
#endif

  this->radio_wrapper_ = std::unique_ptr<ConcreteRadioWrapper>(
      new ConcreteRadioWrapper(*this->radio_, this->board_));

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
    this->mesh_time_synced_ = true;
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

  // Repeaters announce themselves at boot so other nodes can discover
  // them and route through. Companion nodes stay quiet by default —
  // they don't relay traffic, so polluting the air with adverts buys
  // them nothing and just adds congestion.
  if (this->repeater_) {
    this->send_self_advert_();
    this->last_advert_ms_ = millis();
  }

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

  // Periodic self-advert in repeater mode so nodes that boot after we
  // do can discover us without waiting for happenstance traffic.
  if (this->repeater_ && this->advert_interval_sec_ > 0 &&
      now - this->last_advert_ms_ > this->advert_interval_sec_ * 1000U) {
    this->last_advert_ms_ = now;
    this->send_self_advert_();
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
#if defined(USE_SX1262)
  ESP_LOGCONFIG(TAG, "  Pins: SCLK=%d MISO=%d MOSI=%d NSS=%d DIO1=%d RST=%d BUSY=%d",
                P_LORA_SCLK, P_LORA_MISO, P_LORA_MOSI, P_LORA_NSS,
                P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY);
#elif defined(USE_SX1276)
  ESP_LOGCONFIG(TAG, "  Pins: SCLK=%d MISO=%d MOSI=%d NSS=%d DIO0=%d DIO1=%d RST=%d",
                P_LORA_SCLK, P_LORA_MISO, P_LORA_MOSI, P_LORA_NSS,
                P_LORA_DIO_0, (int) P_LORA_DIO_1, P_LORA_RESET);
  if ((int) P_LORA_DIO_1 == RADIOLIB_NC) {
    ESP_LOGCONFIG(TAG, "  (DIO1 not wired, falling back to polling)");
  }
#endif
  ESP_LOGCONFIG(TAG, "  Channels configured: %u", (unsigned) this->channels_.size());
  ESP_LOGCONFIG(TAG, "  Role: %s", this->repeater_ ? "repeater" : "companion");
  if (this->repeater_) {
    if (this->advert_interval_sec_ > 0) {
      ESP_LOGCONFIG(TAG, "  Advert interval: %u s", (unsigned) this->advert_interval_sec_);
    } else {
      ESP_LOGCONFIG(TAG, "  Advert interval: boot only (advert_interval=0)");
    }
  }
  if (this->repeater_) {
    if (this->advert_interval_sec_ == 0) {
      ESP_LOGCONFIG(TAG, "  Advert interval: boot only");
    } else {
      ESP_LOGCONFIG(TAG, "  Advert interval: %u s", (unsigned) this->advert_interval_sec_);
    }
  }
  ESP_LOGCONFIG(TAG, "  Identity source: %s",
                this->static_identity_hex_.empty() ? "NVS (auto)" : "YAML private_key");
  ESP_LOGCONFIG(TAG, "  RTC time: %u (%s)",
                (unsigned) this->rtc_clock_.getCurrentTime(),
                this->mesh_time_synced_
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
  this->mesh_time_synced_ = true;
  ESP_LOGD(TAG, "rtc bumped from mesh: %u -> %u (+%u s)",
           (unsigned) now, (unsigned) mesh_timestamp,
           (unsigned) (mesh_timestamp - now));

  // Persist to NVS so we don't fall back to the hard-coded baseline on
  // the next boot. Cheap because the >60s guard keeps writes rare.
  uint32_t blob = mesh_timestamp;
  this->mesh_time_pref_.save(&blob);
  global_preferences->sync();
}

void MeshCoreComponent::send_self_advert_() {
  // App_data is just the human-readable node name, matching upstream
  // simple_repeater's createSelfAdvert(). Receivers display this in
  // their contact list when auto-add is enabled.
  const auto &name = this->node_name_;
  const size_t name_len = std::min(name.size(), (size_t) MAX_ADVERT_DATA_SIZE);
  mesh::Packet *pkt = this->mesh_->createAdvert(
      this->mesh_->self_id, reinterpret_cast<const uint8_t *>(name.data()), name_len);
  if (pkt == nullptr) {
    ESP_LOGW(TAG, "self-advert: packet allocation failed");
    return;
  }
  this->mesh_->sendFlood(pkt);
  ESP_LOGCONFIG(TAG, "Sent self-advert as '%s'", name.c_str());
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

void EsphomeMesh::onTraceRecv(mesh::Packet *packet, uint32_t tag, uint32_t auth_code,
                              uint8_t flags, const uint8_t *path_snrs,
                              const uint8_t *path_hashes, uint8_t path_len) {
  // Trace packet has reached us as its final destination -- meaning
  // someone wanted to map a route to this node. The path traversal
  // already happened on its way here (each repeater bumped path_snrs
  // forward as it relayed), so by the time we run the originator has
  // its data. We just log what we saw for the operator.
  //
  // path_snrs[i] is each hop's SNR scaled by 4 (so /4.0f for dB).
  // The final SNR (this node's RX) comes from packet->getSNR().
  ESP_LOGD(TAG, "trace tag=%08x auth=%08x flags=%02x hops=%u final_snr=%.1f dB",
           (unsigned) tag, (unsigned) auth_code, (unsigned) flags,
           (unsigned) path_len, packet->getSNR());
  const uint8_t path_sz = flags & 0x03;
  const uint8_t snrs_count = path_len >> path_sz;
  for (uint8_t i = 0; i < snrs_count; i++) {
    ESP_LOGD(TAG, "  trace hop %u: snr=%.2f dB", (unsigned) i,
             ((int8_t) path_snrs[i]) / 4.0f);
  }
}

bool EsphomeMesh::allowPacketForward(const mesh::Packet *packet) {
  // companion role: never forward (upstream default).
  // repeater role: forward; the dispatcher's dedup tables, path-length
  // ceiling (MAX_PATH_SIZE), and air-time budget keep things sane.
  const bool ok = this->owner_->is_repeater();
  if (ok) {
    ESP_LOGD(TAG, "fwd: type=%u path_len=%u",
             (unsigned) packet->getPayloadType(), (unsigned) packet->path_len);
  }
  return ok;
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
