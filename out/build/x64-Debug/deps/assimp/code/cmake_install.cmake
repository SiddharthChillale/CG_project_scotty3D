# Install script for directory: D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code

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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "D:/GM/CompGraphics/Projects/Scotty3D-main/out/build/x64-Debug/deps/assimp/lib/assimp-vc142-mtd.lib")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xassimp-devx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/assimp" TYPE FILE FILES
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/anim.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/aabb.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/ai_assert.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/camera.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/color4.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/color4.inl"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/out/build/x64-Debug/deps/assimp/code/../include/assimp/config.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/ColladaMetaData.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/commonMetaData.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/defs.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/Defines.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/cfileio.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/light.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/material.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/material.inl"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/matrix3x3.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/matrix3x3.inl"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/matrix4x4.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/matrix4x4.inl"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/mesh.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/pbrmaterial.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/postprocess.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/quaternion.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/quaternion.inl"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/scene.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/metadata.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/texture.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/types.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/vector2.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/vector2.inl"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/vector3.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/vector3.inl"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/version.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/cimport.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/importerdesc.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/Importer.hpp"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/DefaultLogger.hpp"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/ProgressHandler.hpp"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/IOStream.hpp"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/IOSystem.hpp"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/Logger.hpp"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/LogStream.hpp"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/NullLogger.hpp"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/cexport.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/Exporter.hpp"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/DefaultIOStream.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/DefaultIOSystem.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/ZipArchiveIOSystem.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/SceneCombiner.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/fast_atof.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/qnan.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/BaseImporter.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/Hash.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/MemoryIOWrapper.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/ParsingUtils.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/StreamReader.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/StreamWriter.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/StringComparison.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/StringUtils.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/SGSpatialSort.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/GenericProperty.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/SpatialSort.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/SkeletonMeshBuilder.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/SmallVector.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/SmoothingGroups.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/SmoothingGroups.inl"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/StandardShapes.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/RemoveComments.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/Subdivision.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/Vertex.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/LineSplitter.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/TinyFormatter.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/Profiler.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/LogAux.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/Bitmap.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/XMLTools.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/IOStreamBuffer.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/CreateAnimMesh.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/irrXMLWrapper.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/BlobIOSystem.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/MathFunctions.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/Exceptional.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/ByteSwapper.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xassimp-devx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/assimp/Compiler" TYPE FILE FILES
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/Compiler/pushpack1.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/Compiler/poppack1.h"
    "D:/GM/CompGraphics/Projects/Scotty3D-main/deps/assimp/code/../include/assimp/Compiler/pstdint.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE FILE FILES "D:/GM/CompGraphics/Projects/Scotty3D-main/out/build/x64-Debug/deps/assimp/code/assimp-vc142-mtd.pdb")
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE FILE FILES "D:/GM/CompGraphics/Projects/Scotty3D-main/out/build/x64-Debug/deps/assimp/code/assimp-vc142-mt.pdb")
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
endif()

