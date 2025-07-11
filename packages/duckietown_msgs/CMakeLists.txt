cmake_minimum_required(VERSION 3.5)
project(duckietown_msgs)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/AntiInstagramHealth.msg"
  "msg/AntiInstagramTransform.msg"
  "msg/AntiInstagramTransformCB.msg"
  "msg/AprilTagDetection.msg"
  "msg/AprilTagDetectionArray.msg"
  "msg/AprilTagsWithInfos.msg"
  "msg/BoolStamped.msg"
  "msg/BoolStampedETHZ17.msg"
  "msg/ButtonEvent.msg"
  "msg/CarControl.msg"
  "msg/CoordinationClearance.msg"
  "msg/CoordinationClearanceETHZ17.msg"
  "msg/CoordinationSignal.msg"
  "msg/CoordinationSignalETHZ17.msg"
  "msg/DisplayFragment.msg"
  "msg/DuckieSensor.msg"
  "msg/FSMState.msg"
  "msg/IntersectionPose.msg"
  "msg/IntersectionPoseImg.msg"
  "msg/IntersectionPoseImgDebug.msg"
  "msg/KinematicsParameters.msg"
  "msg/KinematicsWeights.msg"
  "msg/LanePose.msg"
  "msg/LEDDetection.msg"
  "msg/LEDDetectionArray.msg"
  "msg/LEDDetectionDebugInfo.msg"
  "msg/LEDInterpreter.msg"
  "msg/LEDPattern.msg"
  "msg/MaintenanceState.msg"
  "msg/Num.msg"
  "msg/ObstacleImageDetection.msg"
  "msg/ObstacleImageDetectionList.msg"
  "msg/ObstacleProjectedDetection.msg"
  "msg/ObstacleProjectedDetectionList.msg"
  "msg/ObstacleType.msg"
  "msg/Pixel.msg"
  "msg/Pose2DStamped.msg"
  "msg/Rect.msg"
  "msg/Rects.msg"
  "msg/SceneSegments.msg"
  "msg/Segment.msg"
  "msg/SegmentList.msg"
  "msg/SignalsDetection.msg"
  "msg/SignalsDetectionETHZ17.msg"
  "msg/SourceTargetNodes.msg"
  "msg/StopLineReading.msg"
  "msg/StreetNameDetection.msg"
  "msg/StreetNames.msg"
  "msg/TagInfo.msg"
  "msg/ThetaDotSample.msg"
  "msg/Trajectory.msg"
  "msg/TurnIDandType.msg"
  "msg/Twist2DStamped.msg"
  "msg/Twist2DStampedETHZ17.msg"
  "msg/Vector2D.msg"
  "msg/VehicleCorners.msg"
  "msg/VehiclePose.msg"
  "msg/Vsample.msg"
  "msg/WheelEncoderStamped.msg"
  "msg/WheelsCmd.msg"
  "msg/WheelsCmdStamped.msg"
)

set(srv_files
  "srv/GetVariable.srv"
  "srv/SetFSMState.srv"
  "srv/SetValue.srv"
  "srv/SetVariable.srv"
)


rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs visualization_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
