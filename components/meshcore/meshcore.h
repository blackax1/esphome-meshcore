#pragma once

#include <memory>
#include <string>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

// MeshCore + helpers. Pulled in from PlatformIO via lib_deps.
#include <SPI.h>
#include <Mesh.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/ChannelDetails.h>
#include <helpers/ESP32Board.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/TxtDataHelpers.h>

// Radio variant selection. Exactly one of USE_SX1262 / USE_SX1276 is
// defined by the Python side based on `radio:`.
#if defined(USE_SX1262)
#include <helpers/radiolib/CustomSX1262.h>
#include <helpers/radiolib/CustomSX1262Wrapper.h>
#elif defined(USE_SX1276)
#include <helpers/radiolib/CustomSX1276.h>
#include <helpers/radiolib/CustomSX1276Wrapper.h>
#else
#error "meshcore: no radio variant selected (USE_SX1262 / USE_SX1276 must be defined)"
#endif

#include "esphome/core/preferences.h"

namespace esphome {
namespace meshcore {

class MeshCoreComponent;

/// MeshCore::Mesh subclass. Surfaces interesting events to the owning
/// ESPHome component and answers channel-hash lookups from the dispatcher.
class EsphomeMesh : public mesh::Mesh {
 public:
  EsphomeMesh(MeshCoreComponent *owner, mesh::Radio &radio, mesh::MillisecondClock &ms,
              mesh::RNG &rng, mesh::RTCClock &rtc, mesh::PacketManager &mgr,
              mesh::MeshTables &tables)
      : mesh::Mesh(radio, ms, rng, rtc, mgr, tables), owner_(owner) {}

 protected:
  void onAdvertRecv(mesh::Packet *packet, const mesh::Identity &id, uint32_t timestamp,
                    const uint8_t *app_data, size_t app_data_len) override;
  void onAnonDataRecv(mesh::Packet *packet, const uint8_t *secret, const mesh::Identity &sender,
                      uint8_t *data, size_t len) override;
  int searchChannelsByHash(const uint8_t *hash, mesh::GroupChannel channels[],
                           int max_matches) override;
  void onGroupDataRecv(mesh::Packet *packet, uint8_t type, const mesh::GroupChannel &channel,
                       uint8_t *data, size_t len) override;
  void onTraceRecv(mesh::Packet *packet, uint32_t tag, uint32_t auth_code, uint8_t flags,
                   const uint8_t *path_snrs, const uint8_t *path_hashes,
                   uint8_t path_len) override;

  /// Whether to flood-forward packets we hear. Defaults to false (the
  /// upstream Mesh::allowPacketForward default — companion behaviour).
  /// In repeater role we let everything through; the dispatcher's
  /// dedup tables and path-length checks still keep the network sane.
  bool allowPacketForward(const mesh::Packet *packet) override;

 private:
  MeshCoreComponent *owner_;
};

/// Container for the meshcore text_sensor platform.
class MeshCoreTextSensor : public Component {
 public:
  void set_last_message(text_sensor::TextSensor *s) { this->last_message_ = s; }
  void publish_last_message(const std::string &payload) {
    if (this->last_message_ != nullptr) {
      this->last_message_->publish_state(payload);
    }
  }

 protected:
  text_sensor::TextSensor *last_message_{nullptr};
};

/// Bundle of optional numeric sensors exposed by the meshcore hub.
class MeshCoreSensorBundle : public Component {
 public:
  void set_rssi_sensor(sensor::Sensor *s) { this->rssi_ = s; }
  void set_snr_sensor(sensor::Sensor *s) { this->snr_ = s; }
  void set_battery_sensor(sensor::Sensor *s) { this->battery_ = s; }

  void publish_rssi(float dbm) {
    if (this->rssi_ != nullptr) this->rssi_->publish_state(dbm);
  }
  void publish_snr(float db) {
    if (this->snr_ != nullptr) this->snr_->publish_state(db);
  }
  void publish_battery_voltage(float volts) {
    if (this->battery_ != nullptr) this->battery_->publish_state(volts);
  }

 protected:
  sensor::Sensor *rssi_{nullptr};
  sensor::Sensor *snr_{nullptr};
  sensor::Sensor *battery_{nullptr};
};

/// Hub component: owns the MeshCore stack on the device.
class MeshCoreComponent : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  // After WiFi so logger output during setup() reaches the dashboard.
  // The mesh stack itself is independent of WiFi.
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void set_node_name(const std::string &name) { this->node_name_ = name; }
  void set_static_identity(const std::string &hex) { this->static_identity_hex_ = hex; }
  void set_repeater(bool repeater) { this->repeater_ = repeater; }
  void set_gps_location(float latitude, float longitude) {
    this->gps_lat_ = latitude;
    this->gps_lng_ = longitude;
    this->gps_valid_ = !(std::isnan(latitude) || std::isnan(longitude));
  }
  bool is_repeater() const { return this->repeater_; }
   /// Period in seconds between automatic self-adverts in repeater mode.
   /// 0 means advert only at boot. Companion role ignores this.
   void set_advert_interval(uint32_t seconds) { this->advert_interval_sec_ = seconds; }

  void set_last_message_sensor(MeshCoreTextSensor *s) { this->last_message_sensor_ = s; }
  void set_sensor_bundle(MeshCoreSensorBundle *b) { this->sensors_ = b; }

  /// Register a group channel from YAML. Called once per `channels:` entry
  /// during codegen. `psk_base64` must decode to 16 or 32 bytes.
  void add_channel(const std::string &name, const std::string &psk_base64);

  /// Send `text` on the first configured channel. Returns false if the
  /// mesh isn't ready or no channels are configured.
  bool send_text_message(const std::string &text);

  /// Send `text` on the channel matching `channel_name`. Returns false
  /// if the mesh isn't ready, the channel name is not in the configured
  /// list, or the underlying flood fails.
  bool send_text_message(const std::string &channel_name, const std::string &text);

  /// Adopt `mesh_timestamp` (UNIX epoch seconds, from a received packet)
  /// as our local RTC if it looks more accurate than what we have. The
  /// guard is two-sided: we only accept timestamps that are (a) past
  /// our hard "real world" floor of 1 Jan 2024, and (b) at least
  /// MIN_MESH_TIME_BUMP_SEC newer than the current local time, which
  /// keeps small jitter from constantly re-writing NVS.
  void bump_rtc_from_mesh(uint32_t mesh_timestamp);
  void on_message_received(const std::string &payload, float rssi, float snr);
  const std::vector<ChannelDetails> &channels() const { return this->channels_; }

 protected:
  bool load_or_create_identity_();
  void on_battery_sample_();
  void send_self_advert_();

   std::string node_name_{"esphome-mesh"};
   bool repeater_{false};
   // How often a repeater re-broadcasts its self-advert. 0 = boot only.
   // Default 1800 s (30 min) matches upstream simple_repeater's
   // duty-cycle-conscious cadence. Companion role ignores this.
   uint32_t advert_interval_sec_{1800};
   uint32_t last_advert_ms_{0};
  // Optional hex-encoded static identity from YAML. When non-empty,
  // wins over whatever's cached in NVS.
  std::string static_identity_hex_;

  MeshCoreTextSensor *last_message_sensor_{nullptr};
  MeshCoreSensorBundle *sensors_{nullptr};

  // Pending channel configs supplied during codegen. These are decoded
  // into channels_ during setup() once MeshCore globals are available.
  struct PendingChannel {
    std::string name;
    std::string psk_base64;
  };
  std::vector<PendingChannel> pending_channels_;
  std::vector<ChannelDetails> channels_;

  // MeshCore stack components. Heap-allocated for ones whose constructors
  // need values not available at component construction time.
  ESP32Board board_;
  ESP32RTCClock rtc_clock_;
  ArduinoMillis ms_clock_;
  StdRNG rng_;
  SimpleMeshTables tables_;

  // Concrete radio type chosen at compile time from the YAML radio:
  // selector. RadioLibWrapper is the common base both wrappers share.
#if defined(USE_SX1262)
  using ConcreteRadio = CustomSX1262;
  using ConcreteRadioWrapper = CustomSX1262Wrapper;
#elif defined(USE_SX1276)
  using ConcreteRadio = CustomSX1276;
  using ConcreteRadioWrapper = CustomSX1276Wrapper;
#endif

  std::unique_ptr<ConcreteRadio> radio_;
  std::unique_ptr<ConcreteRadioWrapper> radio_wrapper_;
  std::unique_ptr<StaticPoolPacketManager> packet_mgr_;
   std::unique_ptr<EsphomeMesh> mesh_;

   bool ready_{false};
   bool mesh_time_synced_{false};
   uint32_t last_battery_pub_ms_{0};

  // Preference handle for the persisted identity blob (prv_key + pub_key).
  ESPPreferenceObject identity_pref_;
  // Preference handle for the highest mesh-learned timestamp we've
  // adopted into our RTC. Persisted so the next boot doesn't fall back
  // to MeshCore's hard-coded May 2024 baseline.
  ESPPreferenceObject mesh_time_pref_;

  // GPS location for self-adverts. NaN means no GPS.
  float gps_lat_{std::nanf("")};
  float gps_lng_{std::nanf("")};
  bool gps_valid_{false};
};

}  // namespace meshcore
}  // namespace esphome
