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

#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"

#include "nav2_behavior_tree/utils/test_action_server.hpp"
#include "piper_bt/action/tts_action.hpp"

class TTSActionServer
  : public TestActionServer<audio_common_msgs::action::TTS>
{
public:
  TTSActionServer()
  : TestActionServer("say")
  {
  }

protected:
  void execute(
    const typename std::shared_ptr<
      rclcpp_action::ServerGoalHandle<audio_common_msgs::action::TTS>> goal_handle)
  override
  {
    audio_common_msgs::action::TTS::Result::SharedPtr result =
      std::make_shared<audio_common_msgs::action::TTS::Result>();
    bool return_success = getReturnSuccess();
    if (return_success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }
};

class TTSActionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("tts_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set("node", node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout", std::chrono::milliseconds(20));
    config_->blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration", std::chrono::milliseconds(10));
    config_->blackboard->set<std::chrono::milliseconds>(
      "wait_for_service_timeout", std::chrono::milliseconds(1000));

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<piper_bt::TTSAction>(name, "say", config);
      };

    factory_->registerBuilder<piper_bt::TTSAction>("TTS", builder);
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    server_.reset();
    factory_.reset();
  }

  void SetUp() override
  {
  }

  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<TTSActionServer> server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr TTSActionTestFixture::node_ = nullptr;
std::shared_ptr<TTSActionServer> TTSActionTestFixture::server_ = nullptr;
BT::NodeConfiguration * TTSActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> TTSActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> TTSActionTestFixture::tree_ = nullptr;

TEST_F(TTSActionTestFixture, test_ports)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <TTS/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->getInput<std::string>("language").value(), "en");
  EXPECT_EQ(tree_->rootNode()->getInput<float>("volume").value(), 1.0);
  EXPECT_EQ(tree_->rootNode()->getInput<float>("rate").value(), 1.0);

  xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <TTS text="This is a test" language="es" volume="0.5" rate="0.5"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->getInput<std::string>("text"), "This is a test");
  EXPECT_EQ(tree_->rootNode()->getInput<std::string>("language"), "es");
  EXPECT_EQ(tree_->rootNode()->getInput<float>("volume"), 0.5);
  EXPECT_EQ(tree_->rootNode()->getInput<float>("rate"), 0.5);
}

TEST_F(TTSActionTestFixture, test_tick)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <TTS text="This is a test" />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->getInput<std::string>("text"), "This is a test");

  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize service and spin on new thread
  TTSActionTestFixture::server_ = std::make_shared<TTSActionServer>();
  std::thread server_thread([]() {
      rclcpp::spin(TTSActionTestFixture::server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  server_thread.join();

  std::cout << "All tests passed: " << all_successful << std::endl;

  return all_successful;
}
