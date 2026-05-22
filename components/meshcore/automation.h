#pragma once

#include "esphome/core/automation.h"
#include "meshcore.h"

namespace esphome {
namespace meshcore {

template<typename... Ts> class SendMessageAction : public Action<Ts...> {
 public:
  explicit SendMessageAction(MeshCoreComponent *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(std::string, text)

  void play(Ts... x) override {
    auto text = this->text_.value(x...);
    this->parent_->send_message(text);
  }

 protected:
  MeshCoreComponent *parent_;
};

}  // namespace meshcore
}  // namespace esphome
