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

find_package(PCL 1.7 QUIET REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Boost COMPONENTS program_options)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})


include_directories(${PROJECT_SOURCE_DIR}/include)

export(PACKAGE CPUTSDF)

# create the CPUTSDF library
add_library (cpu_tsdf SHARED
              src/lib/octree.cpp
              src/lib/tsdf_volume_octree.cpp
              src/lib/tsdf_interface.cpp
              src/lib/marching_cubes_tsdf_octree.cpp)

            target_link_libraries (cpu_tsdf ${PCL_LIBRARIES})
install(TARGETS cpu_tsdf DESTINATION lib${LIB_SUFFIX})

# Create the executables
add_executable (tsdf2mesh src/prog/tsdf2mesh.cpp)
target_link_libraries(tsdf2mesh ${PCL_LIBRARIES} cpu_tsdf)
install(TARGETS tsdf2mesh DESTINATION bin)

if (APPLE)
  add_executable (integrate MACOSX_BUNDLE src/prog/integrate.cpp)
else(APPLE)
  add_executable (integrate src/prog/integrate.cpp)
endif(APPLE)
target_link_libraries(integrate ${PCL_LIBRARIES} cpu_tsdf ${Boost_PROGRAM_OPTIONS_LIBRARY})
install(TARGETS integrate DESTINATION bin)

add_executable (get_intrinsics src/prog/get_intrinsics.cpp)
target_link_libraries(get_intrinsics ${PCL_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY})
install(TARGETS get_intrinsics DESTINATION bin)

# Install the Header files preserving the directory
set(HEADER_FILES include/cpu_tsdf/impl/tsdf_volume_octree.hpp
		 include/cpu_tsdf/marching_cubes_tsdf_octree.h
		 include/cpu_tsdf/octree.h
		 include/cpu_tsdf/tsdf_interface.h
		 include/cpu_tsdf/tsdf_volume_octree.h
		 include/eigen_extensions/eigen_extensions.h
)

FOREACH(HEADER ${HEADER_FILES})
	STRING(REGEX MATCH "(.*)[/]" DIR ${HEADER})
	INSTALL(FILES ${HEADER} DESTINATION ${DIR})
ENDFOREACH(HEADER)

# Install CPUTSDF cmake config file for FIND_PACKAGE()
configure_file (${PROJECT_SOURCE_DIR}/CPUTSDFConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/CPUTSDFConfig.cmake @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/CPUTSDFConfig.cmake DESTINATION share/cpu_tsdf)


