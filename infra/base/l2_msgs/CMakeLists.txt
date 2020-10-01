cmake_minimum_required(VERSION 3.5)
project(l2_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


set(msg_files 
  msg/Item.msg 
  msg/L2Actor.msg
  msg/L2ActiveProjects.msg
  msg/L2Event.msg
  msg/L2Role.msg
  msg/Object3D.msg
  msg/Object3DArray.msg
  msg/ProjectItem.msg 
  msg/Project.msg 
  msg/Projects.msg
  msg/L2Sequence.msg
  msg/L2SequenceArray.msg
  msg/L2SequenceItem.msg)
set(srv_files 
  srv/SpawnObject3D.srv
  srv/RemoveObject3D.srv
  srv/PushParameter.srv
  srv/GetFile.srv
  srv/PutFile.srv
  srv/ListFiles.srv
  srv/GetProject.srv
  srv/GetObject3D.srv)
  srv/Object3D.srv
  srv/L2ActiveProjects.srv
  srv/L2ProjectItem.srv
  srv/L2Project.srv
  srv/L2Actor.srv
  srv/L2SearchProjects.srv)
set(action_files
  action/L2Sequence.action)
rosidl_generate_interfaces( ${PROJECT_NAME} ${msg_files} ${srv_files} ${action_files} DEPENDENCIES builtin_interfaces std_msgs diagnostic_msgs geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
