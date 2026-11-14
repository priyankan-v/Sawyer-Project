cmake_minimum_required(VERSION 3.16)
project(irobot_sawyer_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CalibrationCommand.action"
  "action/InteractionSession.action"
  "action/JointStream.action"
  "action/MotionCommand.action"
  "action/MoveCartesian.action"
  "msg/CalibrationCommandFeedback.msg"
  "msg/CalibrationCommandGoal.msg"
  "msg/CalibrationCommandResult.msg"
  "msg/DigitalIOState.msg"
  "msg/EndpointState.msg"
  "msg/EndpointStates.msg"
  "msg/IOComponentConfiguration.msg"
  "msg/IOComponentStatus.msg"
  "msg/IODataStatus.msg"
  "msg/IODeviceConfiguration.msg"
  "msg/IODeviceStatus.msg"
  "msg/IOStatus.msg"
  "msg/InteractionControlCommand.msg"
  "msg/InteractionControlState.msg"
  "msg/JointLimits.msg"
  "msg/MotionCommandFeedback.msg"
  "msg/MotionCommandGoal.msg"
  "msg/MotionCommandResult.msg"
  "msg/MotionStatus.msg"
  "msg/RobotAssemblyState.msg"
  "msg/StreamCommand.msg"
  "msg/TrackingOptions.msg"
  "msg/Trajectory.msg"
  "msg/TrajectoryAnalysis.msg"
  "msg/TrajectoryOptions.msg"
  "msg/Waypoint.msg"
  "msg/WaypointOptions.msg"
  "srv/CheckJointCollision.srv"
  "srv/ForwardKinematics.srv"
  "srv/InverseKinematics.srv"
  "srv/ReadParameter.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs
)
install(FILES LICENSE NOTICE DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
