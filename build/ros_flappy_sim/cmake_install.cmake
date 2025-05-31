# Install script for directory: /Users/mostafalotfy/Documents/University/Master/SSLab/sim_ws/src/ros_flappy_sim

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/Users/mostafalotfy/Documents/University/Master/SSLab/sim_ws/install")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_flappy_sim/cmake" TYPE FILE FILES "/Users/mostafalotfy/Documents/University/Master/SSLab/sim_ws/build/ros_flappy_sim/catkin_generated/installspace/ros_flappy_sim-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/Users/mostafalotfy/Documents/University/Master/SSLab/sim_ws/devel/include/ros_flappy_sim")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/Users/mostafalotfy/Documents/University/Master/SSLab/sim_ws/devel/share/roseus/ros/ros_flappy_sim")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/Users/mostafalotfy/Documents/University/Master/SSLab/sim_ws/devel/share/common-lisp/ros/ros_flappy_sim")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/Users/mostafalotfy/Documents/University/Master/SSLab/sim_ws/devel/share/gennodejs/ros/ros_flappy_sim")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/Users/mostafalotfy/anaconda3/envs/ROS/bin/python3.9" -m compileall "/Users/mostafalotfy/Documents/University/Master/SSLab/sim_ws/devel/lib/python3.9/site-packages/ros_flappy_sim")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.9/site-packages" TYPE DIRECTORY FILES "/Users/mostafalotfy/Documents/University/Master/SSLab/sim_ws/devel/lib/python3.9/site-packages/ros_flappy_sim")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/Users/mostafalotfy/Documents/University/Master/SSLab/sim_ws/build/ros_flappy_sim/catkin_generated/installspace/ros_flappy_sim.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_flappy_sim/cmake" TYPE FILE FILES "/Users/mostafalotfy/Documents/University/Master/SSLab/sim_ws/build/ros_flappy_sim/catkin_generated/installspace/ros_flappy_sim-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_flappy_sim/cmake" TYPE FILE FILES
    "/Users/mostafalotfy/Documents/University/Master/SSLab/sim_ws/build/ros_flappy_sim/catkin_generated/installspace/ros_flappy_simConfig.cmake"
    "/Users/mostafalotfy/Documents/University/Master/SSLab/sim_ws/build/ros_flappy_sim/catkin_generated/installspace/ros_flappy_simConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_flappy_sim" TYPE FILE FILES "/Users/mostafalotfy/Documents/University/Master/SSLab/sim_ws/src/ros_flappy_sim/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_flappy_sim/launch" TYPE DIRECTORY FILES "/Users/mostafalotfy/Documents/University/Master/SSLab/sim_ws/src/ros_flappy_sim/launch/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_flappy_sim/msg" TYPE DIRECTORY FILES "/Users/mostafalotfy/Documents/University/Master/SSLab/sim_ws/src/ros_flappy_sim/msg/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_flappy_sim/scripts" TYPE DIRECTORY FILES "/Users/mostafalotfy/Documents/University/Master/SSLab/sim_ws/src/ros_flappy_sim/scripts/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_flappy_sim/src/mujoco_ros_bridge" TYPE DIRECTORY FILES "/Users/mostafalotfy/Documents/University/Master/SSLab/sim_ws/src/ros_flappy_sim/src/mujoco_ros_bridge/")
endif()

