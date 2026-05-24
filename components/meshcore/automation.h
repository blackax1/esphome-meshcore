#pragma once

#include "esphome/core/automation.h"
#include "meshcore.h"

namespace esphome {
namespace meshcore {

/// Send a UTF-8 text message on a configured group channel.
///
/// `channel` is optional; when omitted the message goes out on the first
/// configured channel (matches the prior `meshcore.send_message`
/// behaviour). `text` is mandatory and templatable.
template<typename... Ts> class SendTextMessageAction : public Action<Ts...> {
 public:
  explicit SendTextMessageAction(MeshCoreComponent *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(std::string, text)
  TEMPLATABLE_VALUE(std::string, channel)

  void play(Ts... x) override {
    const auto text = this->text_.value(x...);
    const auto channel = this->channel_.value(x...);
    if (channel.empty()) {
      this->parent_->send_text_message(text);
    } else {
      this->parent_->send_text_message(channel, text);
    }
  }

 protected:
  MeshCoreComponent *parent_;
};

}  // namespace meshcore
}  // namespace esphome
