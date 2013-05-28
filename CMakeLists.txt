cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cpu_tsdf)
# Find modules
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
set(CMAKE_BUILD_TYPE Release CACHE STRING "Release" FORCE)

# Use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH  FALSE)


IF (NOT APPLE)
  find_package(OpenMP)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
ENDIF (NOT APPLE)

set(LIB_SUFFIX CACHE STRING "suffix for the library directory need for x86-64 systems that use lib64 ")

# The RPATH to be used when installing
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}")
# Add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(PCL 1.7 QUIET)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


include_directories(${PROJECT_SOURCE_DIR}/include)

add_library (cpu_tsdf SHARED
              src/lib/octree.cpp
              src/lib/tsdf_volume_octree.cpp
              src/lib/tsdf_interface.cpp
              src/lib/marching_cubes_tsdf_octree.cpp)

            target_link_libraries (cpu_tsdf ${PCL_LIBRARIES})
install(TARGETS cpu_tsdf DESTINATION lib${LIB_SUFFIX})

add_executable (tsdf2mesh src/prog/tsdf2mesh.cpp)
target_link_libraries(tsdf2mesh ${PCL_LIBRARIES} cpu_tsdf)
install(TARGETS tsdf2mesh DESTINATION bin)
configure_file (${PROJECT_SOURCE_DIR}/CPUTSDFConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/CPUTSDFConfig.cmake)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/CPUTSDFConfig.cmake DESTINATION share)



