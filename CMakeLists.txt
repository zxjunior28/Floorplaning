# declare cmake version
cmake_minimum_required (VERSION 3.16)

# set main project infomation
project (floorplaning VERSION 0.0.1 LANGUAGES CXX)


# add module search path
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/tools/cmake)

# set build examples 
option(BUILD_EXAMPLES "Set build examples (within each modules)" OFF)

# import utilities
include(utilities)

# set global cxx standard as C++17
set_global_cxx_standard(17)

# set global windows export all symbols
set_global_windows_export_all_symbols()

# set global build output name same as install
set_global_build_output_name_same_as_install()

# set global rpath
set_global_rpath()

# add source
add_subdirectory(src)

# add example
if(BUILD_EXAMPLES)
	add_subdirectory(examples)
endif()
