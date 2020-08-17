cmake_minimum_required(VERSION 2.8.12)
project(ipab_controller_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  EffortFeedforwardWithJointFeedback.msg
  EffortFeedforwardWithJointFeedbackTrajectory.msg
  FeedbackPolicy.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
