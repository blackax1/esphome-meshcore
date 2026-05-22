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
#include <helpers/radiolib/CustomSX1262.h>
#include <helpers/radiolib/CustomSX1262Wrapper.h>

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

  void set_last_message_sensor(MeshCoreTextSensor *s) { this->last_message_sensor_ = s; }
  void set_sensor_bundle(MeshCoreSensorBundle *b) { this->sensors_ = b; }

  /// Register a group channel from YAML. Called once per `channels:` entry
  /// during codegen. `psk_base64` must decode to 16 or 32 bytes.
  void add_channel(const std::string &name, const std::string &psk_base64);

  /// Send `text` on the first configured channel. Returns false if the
  /// mesh isn't ready or no channels are configured.
  bool send_message(const std::string &text);

  // Called from EsphomeMesh callbacks.
  void on_message_received(const std::string &payload, float rssi, float snr);
  const std::vector<ChannelDetails> &channels() const { return this->channels_; }

 protected:
  bool load_or_create_identity_();
  void on_battery_sample_();

  std::string node_name_{"esphome-mesh"};
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

  std::unique_ptr<CustomSX1262> radio_;
  std::unique_ptr<CustomSX1262Wrapper> radio_wrapper_;
  std::unique_ptr<StaticPoolPacketManager> packet_mgr_;
  std::unique_ptr<EsphomeMesh> mesh_;

  bool ready_{false};
  uint32_t last_battery_pub_ms_{0};

  // Preference handle for the persisted identity blob (prv_key + pub_key).
  ESPPreferenceObject identity_pref_;
};

}  // namespace meshcore
}  // namespace esphome
