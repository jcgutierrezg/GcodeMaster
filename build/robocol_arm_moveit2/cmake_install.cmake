# Install script for directory: /home/jcgg/robocol_ws/GcodeMaster/src/robocol_arm_moveit2

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/jcgg/robocol_ws/GcodeMaster/install/robocol_arm_moveit2")
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
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/main_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/main_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/main_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2" TYPE EXECUTABLE FILES "/home/jcgg/robocol_ws/GcodeMaster/build/robocol_arm_moveit2/main_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/main_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/main_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/main_node"
         OLD_RPATH "/home/jcgg/ws_moveit/install/moveit_ros_planning_interface/lib:/home/jcgg/ws_moveit/install/moveit_ros_move_group/lib:/home/jcgg/ws_moveit/install/moveit_core/lib:/home/jcgg/ws_moveit/install/moveit_msgs/lib:/opt/ros/humble/lib:/home/jcgg/ws_moveit/install/moveit_ros_warehouse/lib:/home/jcgg/ws_moveit/install/moveit_ros_planning/lib:/home/jcgg/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:/opt/ros/humble/lib/x86_64-linux-gnu:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/main_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/planning_test" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/planning_test")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/planning_test"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2" TYPE EXECUTABLE FILES "/home/jcgg/robocol_ws/GcodeMaster/build/robocol_arm_moveit2/planning_test")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/planning_test" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/planning_test")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/planning_test"
         OLD_RPATH "/home/jcgg/ws_moveit/install/moveit_ros_planning_interface/lib:/home/jcgg/ws_moveit/install/moveit_visual_tools/lib:/home/jcgg/ws_moveit/install/moveit_ros_move_group/lib:/home/jcgg/ws_moveit/install/moveit_core/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/home/jcgg/ws_moveit/install/moveit_msgs/lib:/opt/ros/humble/lib:/home/jcgg/ws_moveit/install/moveit_ros_warehouse/lib:/home/jcgg/ws_moveit/install/moveit_ros_planning/lib:/home/jcgg/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:/home/jcgg/ws_moveit/install/rviz_visual_tools/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/planning_test")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/plan_2_arm" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/plan_2_arm")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/plan_2_arm"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2" TYPE EXECUTABLE FILES "/home/jcgg/robocol_ws/GcodeMaster/build/robocol_arm_moveit2/plan_2_arm")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/plan_2_arm" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/plan_2_arm")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/plan_2_arm"
         OLD_RPATH "/home/jcgg/ws_moveit/install/moveit_ros_planning_interface/lib:/home/jcgg/ws_moveit/install/moveit_visual_tools/lib:/home/jcgg/ws_moveit/install/moveit_ros_move_group/lib:/home/jcgg/ws_moveit/install/moveit_core/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/home/jcgg/ws_moveit/install/moveit_msgs/lib:/opt/ros/humble/lib:/home/jcgg/ws_moveit/install/moveit_ros_warehouse/lib:/home/jcgg/ws_moveit/install/moveit_ros_planning/lib:/home/jcgg/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:/home/jcgg/ws_moveit/install/rviz_visual_tools/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/plan_2_arm")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/traj_extractor" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/traj_extractor")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/traj_extractor"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2" TYPE EXECUTABLE FILES "/home/jcgg/robocol_ws/GcodeMaster/build/robocol_arm_moveit2/traj_extractor")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/traj_extractor" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/traj_extractor")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/traj_extractor"
         OLD_RPATH "/home/jcgg/ws_moveit/install/moveit_ros_planning_interface/lib:/home/jcgg/ws_moveit/install/moveit_visual_tools/lib:/home/jcgg/ws_moveit/install/moveit_ros_move_group/lib:/home/jcgg/ws_moveit/install/moveit_core/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/home/jcgg/ws_moveit/install/moveit_msgs/lib:/opt/ros/humble/lib:/home/jcgg/ws_moveit/install/moveit_ros_warehouse/lib:/home/jcgg/ws_moveit/install/moveit_ros_planning/lib:/home/jcgg/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:/home/jcgg/ws_moveit/install/rviz_visual_tools/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/traj_extractor")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/pilz_planning" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/pilz_planning")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/pilz_planning"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2" TYPE EXECUTABLE FILES "/home/jcgg/robocol_ws/GcodeMaster/build/robocol_arm_moveit2/pilz_planning")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/pilz_planning" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/pilz_planning")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/pilz_planning"
         OLD_RPATH "/home/jcgg/ws_moveit/install/moveit_ros_planning_interface/lib:/home/jcgg/ws_moveit/install/moveit_visual_tools/lib:/home/jcgg/ws_moveit/install/moveit_ros_move_group/lib:/home/jcgg/ws_moveit/install/moveit_core/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/home/jcgg/ws_moveit/install/moveit_msgs/lib:/opt/ros/humble/lib:/home/jcgg/ws_moveit/install/moveit_ros_warehouse/lib:/home/jcgg/ws_moveit/install/moveit_ros_planning/lib:/home/jcgg/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:/home/jcgg/ws_moveit/install/rviz_visual_tools/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robocol_arm_moveit2/pilz_planning")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robocol_arm_moveit2" TYPE DIRECTORY FILES
    "/home/jcgg/robocol_ws/GcodeMaster/src/robocol_arm_moveit2/meshes"
    "/home/jcgg/robocol_ws/GcodeMaster/src/robocol_arm_moveit2/urdf"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/jcgg/robocol_ws/GcodeMaster/build/robocol_arm_moveit2/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/robocol_arm_moveit2")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/jcgg/robocol_ws/GcodeMaster/build/robocol_arm_moveit2/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/robocol_arm_moveit2")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robocol_arm_moveit2/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robocol_arm_moveit2/environment" TYPE FILE FILES "/home/jcgg/robocol_ws/GcodeMaster/build/robocol_arm_moveit2/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robocol_arm_moveit2/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robocol_arm_moveit2/environment" TYPE FILE FILES "/home/jcgg/robocol_ws/GcodeMaster/build/robocol_arm_moveit2/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robocol_arm_moveit2" TYPE FILE FILES "/home/jcgg/robocol_ws/GcodeMaster/build/robocol_arm_moveit2/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robocol_arm_moveit2" TYPE FILE FILES "/home/jcgg/robocol_ws/GcodeMaster/build/robocol_arm_moveit2/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robocol_arm_moveit2" TYPE FILE FILES "/home/jcgg/robocol_ws/GcodeMaster/build/robocol_arm_moveit2/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robocol_arm_moveit2" TYPE FILE FILES "/home/jcgg/robocol_ws/GcodeMaster/build/robocol_arm_moveit2/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robocol_arm_moveit2" TYPE FILE FILES "/home/jcgg/robocol_ws/GcodeMaster/build/robocol_arm_moveit2/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/jcgg/robocol_ws/GcodeMaster/build/robocol_arm_moveit2/ament_cmake_index/share/ament_index/resource_index/packages/robocol_arm_moveit2")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robocol_arm_moveit2/cmake" TYPE FILE FILES
    "/home/jcgg/robocol_ws/GcodeMaster/build/robocol_arm_moveit2/ament_cmake_core/robocol_arm_moveit2Config.cmake"
    "/home/jcgg/robocol_ws/GcodeMaster/build/robocol_arm_moveit2/ament_cmake_core/robocol_arm_moveit2Config-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robocol_arm_moveit2" TYPE FILE FILES "/home/jcgg/robocol_ws/GcodeMaster/src/robocol_arm_moveit2/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/jcgg/robocol_ws/GcodeMaster/build/robocol_arm_moveit2/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
