# Install script for directory: /home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps_install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/sophus/cmake/SophusTargets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/sophus/cmake/SophusTargets.cmake"
         "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/build_r2a/CMakeFiles/Export/share/sophus/cmake/SophusTargets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/sophus/cmake/SophusTargets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/sophus/cmake/SophusTargets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sophus/cmake" TYPE FILE FILES "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/build_r2a/CMakeFiles/Export/share/sophus/cmake/SophusTargets.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sophus/cmake" TYPE FILE FILES
    "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/build_r2a/SophusConfig.cmake"
    "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/build_r2a/SophusConfigVersion.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/sophus" TYPE FILE FILES
    "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/sophus/average.hpp"
    "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/sophus/cartesian.hpp"
    "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/sophus/ceres_local_parameterization.hpp"
    "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/sophus/ceres_manifold.hpp"
    "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/sophus/ceres_typetraits.hpp"
    "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/sophus/common.hpp"
    "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/sophus/geometry.hpp"
    "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/sophus/interpolate.hpp"
    "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/sophus/interpolate_details.hpp"
    "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/sophus/num_diff.hpp"
    "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/sophus/rotation_matrix.hpp"
    "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/sophus/rxso2.hpp"
    "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/sophus/rxso3.hpp"
    "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/sophus/se2.hpp"
    "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/sophus/se3.hpp"
    "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/sophus/sim2.hpp"
    "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/sophus/sim3.hpp"
    "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/sophus/sim_details.hpp"
    "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/sophus/so2.hpp"
    "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/sophus/so3.hpp"
    "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/sophus/spline.hpp"
    "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/sophus/types.hpp"
    "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/sophus/velocities.hpp"
    )
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/ysc/lite3_patrol_project/fast_livo2_ws/deps/Sophus-1.22.10/build_r2a/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
