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

#ifndef PIPER_BT__ACTION__TTS_ACTION_HPP_
#define PIPER_BT__ACTION__TTS_ACTION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "audio_common_msgs/action/tts.hpp"

namespace piper_bt
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps audio_common_msgs::action::TTS
 */
class TTSAction : public nav2_behavior_tree::BtActionNode<audio_common_msgs::action::TTS>
{
public:
  /**
   * @brief A constructor for piper_bt::TTSAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  TTSAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   * @return BT::NodeStatus Status of tick execution
   */
  void on_tick() override;

  /**
   * @brief Function to perform some user-defined operation upon successful completion of the action
   */
  BT::NodeStatus on_success() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>("text", "Text to play"),
        BT::InputPort<std::string>("language", "en", "Language to use"),
        BT::InputPort<float>("volume", 1.0, "Volume of the speech"),
        BT::InputPort<float>("rate", 1.0, "Rate of the speech"),
      });
  }
};

}  // namespace piper_bt

#endif  // PIPER_BT__ACTION__TTS_ACTION_HPP_
