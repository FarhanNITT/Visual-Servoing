# Install script for directory: /home/farhan/ros2_gazebo/src/rrbot_gazebo

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/farhan/ros2_gazebo/install/rrbot_gazebo")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrbot_gazebo/publisher" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrbot_gazebo/publisher")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrbot_gazebo/publisher"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/rrbot_gazebo" TYPE EXECUTABLE FILES "/home/farhan/ros2_gazebo/build/rrbot_gazebo/publisher")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrbot_gazebo/publisher" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrbot_gazebo/publisher")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrbot_gazebo/publisher"
         OLD_RPATH "/home/farhan/ros2_humble/install/rclcpp/lib:/home/farhan/ros2_humble/install/std_msgs/lib:/home/farhan/ros2_humble/install/libstatistics_collector/lib:/home/farhan/ros2_humble/install/rcl/lib:/home/farhan/ros2_humble/install/rmw_implementation/lib:/home/farhan/ros2_humble/install/ament_index_cpp/lib:/home/farhan/ros2_humble/install/rcl_logging_spdlog/lib:/home/farhan/ros2_humble/install/rcl_logging_interface/lib:/home/farhan/ros2_humble/install/rcl_interfaces/lib:/home/farhan/ros2_humble/install/rcl_yaml_param_parser/lib:/home/farhan/ros2_humble/install/libyaml_vendor/lib:/home/farhan/ros2_humble/install/rosgraph_msgs/lib:/home/farhan/ros2_humble/install/statistics_msgs/lib:/home/farhan/ros2_humble/install/tracetools/lib:/home/farhan/ros2_humble/install/builtin_interfaces/lib:/home/farhan/ros2_humble/install/rosidl_typesupport_fastrtps_c/lib:/home/farhan/ros2_humble/install/rosidl_typesupport_fastrtps_cpp/lib:/home/farhan/ros2_humble/install/fastcdr/lib:/home/farhan/ros2_humble/install/rmw/lib:/home/farhan/ros2_humble/install/rosidl_typesupport_introspection_cpp/lib:/home/farhan/ros2_humble/install/rosidl_typesupport_introspection_c/lib:/home/farhan/ros2_humble/install/rosidl_typesupport_cpp/lib:/home/farhan/ros2_humble/install/rosidl_typesupport_c/lib:/home/farhan/ros2_humble/install/rcpputils/lib:/home/farhan/ros2_humble/install/rosidl_runtime_c/lib:/home/farhan/ros2_humble/install/rcutils/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrbot_gazebo/publisher")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrbot_gazebo/switch" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrbot_gazebo/switch")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrbot_gazebo/switch"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/rrbot_gazebo" TYPE EXECUTABLE FILES "/home/farhan/ros2_gazebo/build/rrbot_gazebo/switch")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrbot_gazebo/switch" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrbot_gazebo/switch")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrbot_gazebo/switch"
         OLD_RPATH "/home/farhan/ros2_humble/install/rclcpp/lib:/home/farhan/ros2_humble/install/libstatistics_collector/lib:/home/farhan/ros2_humble/install/rcl/lib:/home/farhan/ros2_humble/install/rmw_implementation/lib:/home/farhan/ros2_humble/install/ament_index_cpp/lib:/home/farhan/ros2_humble/install/rcl_logging_spdlog/lib:/home/farhan/ros2_humble/install/rcl_logging_interface/lib:/home/farhan/ros2_humble/install/rcl_interfaces/lib:/home/farhan/ros2_humble/install/rcl_yaml_param_parser/lib:/home/farhan/ros2_humble/install/libyaml_vendor/lib:/home/farhan/ros2_humble/install/rosgraph_msgs/lib:/home/farhan/ros2_humble/install/statistics_msgs/lib:/home/farhan/ros2_humble/install/tracetools/lib:/home/farhan/ros2_humble/install/builtin_interfaces/lib:/home/farhan/ros2_humble/install/lifecycle_msgs/lib:/home/farhan/ros2_humble/install/rosidl_typesupport_fastrtps_c/lib:/home/farhan/ros2_humble/install/rosidl_typesupport_fastrtps_cpp/lib:/home/farhan/ros2_humble/install/fastcdr/lib:/home/farhan/ros2_humble/install/rmw/lib:/home/farhan/ros2_humble/install/rosidl_typesupport_introspection_cpp/lib:/home/farhan/ros2_humble/install/rosidl_typesupport_introspection_c/lib:/home/farhan/ros2_humble/install/rosidl_typesupport_cpp/lib:/home/farhan/ros2_humble/install/rosidl_typesupport_c/lib:/home/farhan/ros2_humble/install/rcpputils/lib:/home/farhan/ros2_humble/install/rosidl_runtime_c/lib:/home/farhan/ros2_humble/install/rcutils/lib:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrbot_gazebo/switch")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rrbot_gazebo" TYPE DIRECTORY FILES "/home/farhan/ros2_gazebo/src/rrbot_gazebo/launch")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rrbot_gazebo" TYPE DIRECTORY FILES "/home/farhan/ros2_gazebo/src/rrbot_gazebo/worlds")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rrbot_gazebo" TYPE DIRECTORY FILES "/home/farhan/ros2_gazebo/src/rrbot_gazebo/config")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/farhan/ros2_gazebo/build/rrbot_gazebo/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/rrbot_gazebo")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/farhan/ros2_gazebo/build/rrbot_gazebo/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/rrbot_gazebo")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rrbot_gazebo/environment" TYPE FILE FILES "/home/farhan/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rrbot_gazebo/environment" TYPE FILE FILES "/home/farhan/ros2_gazebo/build/rrbot_gazebo/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rrbot_gazebo/environment" TYPE FILE FILES "/home/farhan/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rrbot_gazebo/environment" TYPE FILE FILES "/home/farhan/ros2_gazebo/build/rrbot_gazebo/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rrbot_gazebo" TYPE FILE FILES "/home/farhan/ros2_gazebo/build/rrbot_gazebo/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rrbot_gazebo" TYPE FILE FILES "/home/farhan/ros2_gazebo/build/rrbot_gazebo/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rrbot_gazebo" TYPE FILE FILES "/home/farhan/ros2_gazebo/build/rrbot_gazebo/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rrbot_gazebo" TYPE FILE FILES "/home/farhan/ros2_gazebo/build/rrbot_gazebo/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rrbot_gazebo" TYPE FILE FILES "/home/farhan/ros2_gazebo/build/rrbot_gazebo/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/farhan/ros2_gazebo/build/rrbot_gazebo/ament_cmake_index/share/ament_index/resource_index/packages/rrbot_gazebo")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rrbot_gazebo/cmake" TYPE FILE FILES
    "/home/farhan/ros2_gazebo/build/rrbot_gazebo/ament_cmake_core/rrbot_gazeboConfig.cmake"
    "/home/farhan/ros2_gazebo/build/rrbot_gazebo/ament_cmake_core/rrbot_gazeboConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rrbot_gazebo" TYPE FILE FILES "/home/farhan/ros2_gazebo/src/rrbot_gazebo/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/farhan/ros2_gazebo/build/rrbot_gazebo/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
