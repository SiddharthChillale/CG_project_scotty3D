IF(NOT EXISTS "D:/GM/CompGraphics/Projects/Scotty3D-main/out/build/x64-Debug/deps/assimp/install_manifest.txt")
  MESSAGE(FATAL_ERROR "Cannot find install manifest: \"D:/GM/CompGraphics/Projects/Scotty3D-main/out/build/x64-Debug/deps/assimp/install_manifest.txt\"")
ENDIF(NOT EXISTS "D:/GM/CompGraphics/Projects/Scotty3D-main/out/build/x64-Debug/deps/assimp/install_manifest.txt")

FILE(READ "D:/GM/CompGraphics/Projects/Scotty3D-main/out/build/x64-Debug/deps/assimp/install_manifest.txt" files)
STRING(REGEX REPLACE "\n" ";" files "${files}")
FOREACH(file ${files})
  MESSAGE(STATUS "Uninstalling \"$ENV{DESTDIR}${file}\"")
  EXEC_PROGRAM(
    "C:/Program Files (x86)/Microsoft Visual Studio/2019/Community/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe" ARGS "-E remove \"$ENV{DESTDIR}${file}\""
    OUTPUT_VARIABLE rm_out
    RETURN_VALUE rm_retval
    )
  IF(NOT "${rm_retval}" STREQUAL 0)
    MESSAGE(FATAL_ERROR "Problem when removing \"$ENV{DESTDIR}${file}\"")
  ENDIF(NOT "${rm_retval}" STREQUAL 0)
ENDFOREACH(file)
