// Copyright (c) 2025 Alberto J. Tudela Roldán
// Copyright (c) 2025 Grupo Avispa, DTE, Universidad de Málaga
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <memory>

#include "piper_bt/action/tts_action.hpp"

namespace piper_bt
{

TTSAction::TTSAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<audio_common_msgs::action::TTS>(xml_tag_name, action_name, conf)
{
}

void TTSAction::on_tick()
{
  std::string text;
  getInput("text", text);
  std::string language;
  getInput("language", language);
  float volume;
  getInput("volume", volume);
  float rate;
  getInput("rate", rate);

  goal_.text = text;
  goal_.language = language;
  goal_.volume = volume;
  goal_.rate = rate;
}

BT::NodeStatus TTSAction::on_success()
{
  return BT::NodeStatus::SUCCESS;
}

}  // namespace piper_bt

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<piper_bt::TTSAction>(name, "say", config);
    };

  factory.registerBuilder<piper_bt::TTSAction>("TTS", builder);
}
