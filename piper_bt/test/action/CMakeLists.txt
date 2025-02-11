find_package(ament_index_cpp REQUIRED)

# Test for the tts action
ament_add_gtest(test_tts_action
  test_tts_action.cpp
)
target_link_libraries(test_tts_action
  ament_index_cpp::ament_index_cpp
  piper_tts_action_bt_node
)
ament_target_dependencies(test_tts_action
  ${dependencies}
)
