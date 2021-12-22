# Install script for directory: D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "D:/GM/CompGraphics/Projects/Scotty3D-main/out/install/x64-Debug")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xlibassimp5.0.1-devx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/assimp-5.0" TYPE FILE FILES
    "D:/GM/CompGraphics/Projects/Scotty3D-main/out/build/x64-Debug/deps/assimp/assimp-config.cmake"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/out/build/x64-Debug/deps/assimp/assimp-config-version.cmake"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/out/build/x64-Debug/deps/assimp/assimpTargets.cmake"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/out/build/x64-Debug/deps/assimp/assimpTargets-debug.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("D:/GM/CompGraphics/Projects/Scotty3D-main/out/build/x64-Debug/deps/assimp/contrib/zlib/cmake_install.cmake")
  include("D:/GM/CompGraphics/Projects/Scotty3D-main/out/build/x64-Debug/deps/assimp/contrib/cmake_install.cmake")
  include("D:/GM/CompGraphics/Projects/Scotty3D-main/out/build/x64-Debug/deps/assimp/code/cmake_install.cmake")

endif()

