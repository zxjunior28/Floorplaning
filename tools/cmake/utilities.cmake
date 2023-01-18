# set global cxx standard
macro(set_global_cxx_standard version)
    set(CMAKE_CXX_STANDARD ${version})
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
    message(STATUS "${PROJECT_NAME}: C++${version} standard global supported!!")
endmacro()

# set global windows export all symbols
macro(set_global_windows_export_all_symbols)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    message(STATUS "${PROJECT_NAME}: Windows export all symnols global supported!!")
endmacro()

# set global position independent code (PIC)
macro(set_global_position_independent_code)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
    message(STATUS "${PROJECT_NAME}: Enable position independent code (PIC)!!")
endmacro()

# set global interprocedural optimization (IPO/LTO)
macro(set_global_interprocedural_optimization)
    include(CheckIPOSupported)
    check_ipo_supported(RESULT lto_result OUTPUT lto_output)
    if(lto_result)
        set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
        message(STATUS "${PROJECT_NAME}: Enable interprocedural_optimization (IPO/LTO)!!")
    else()
        message(STATUS "${PROJECT_NAME}: Interprocedural_optimization (IPO/LTO) is not supported: ${output}")
    endif()
endmacro()

# set global build output name same as install
macro(set_global_build_output_name_same_as_install)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
endmacro()

# set global rpath
macro(set_global_rpath)
    if(UNIX)
        set(CMAKE_SKIP_BUILD_RPATH FALSE)
        set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
        set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
        if(NOT APPLE)
            set(CMAKE_INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib;$ORIGIN/../Lib")
        endif()
    endif()
endmacro()

# get subdirectory list (with CmakeLists.txt)
function(get_subdirectories_name subdirs_name current_dir)
    file(GLOB results ${current_dir}/* )
    set(tmp_list "")
    foreach(item ${results})
        if(IS_DIRECTORY ${item} AND EXISTS "${item}/CMakeLists.txt")
            get_filename_component(subdir_name ${item} NAME)
            list(APPEND tmp_list ${subdir_name})
        endif()
    endforeach()
    set(${subdirs_name} ${tmp_list} PARENT_SCOPE)
endfunction()

# find google test
macro(find_googletest)
    # enable testing
    enable_testing()
    
    # find GTest
    find_package(GTest QUIET)

    # fetch content from github
    if(NOT ${GTest_FOUND})
        message(STATUS "${PROJECT_NAME}: Fetch GoogleTest contents from GitHub!")
        include(FetchContent)
        FetchContent_Declare(
            googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG        release-1.11.0
        )
        FetchContent_MakeAvailable(googletest)
        find_package(GTest QUIET)
    endif()

    # check google test succeed
    if(TARGET GTest::gtest) 
        # install cmake GoogleTest module
        include(GoogleTest)
        # enable testing
        enable_testing()
        # disable install GoogleTest
        set(INSTALL_GTEST OFF CACHE BOOL " " FORCE)
        # mark some GoogleTest setting as advanced
        mark_as_advanced(
            BUILD_GMOCK BUILD_GTEST BUILD_SHARED_LIBS
            gmock_build_tests gtest_build_samples 
            gtest_build_tests gtest_disable_pthreads 
            gtest_force_shared_crt gtest_hide_internal_symbols
        )
        message(STATUS "${PROJECT_NAME}: GoogleTest found and add succeed!")
    else()
        message(FATAL_ERROR "${PROJECT_NAME}: GoogleTest not found!")
    endif()
endmacro()

# add google test target with sources and libraries
function(add_gtest_with_libraries target_name sources libraries)
    add_executable(${target_name} ${sources})
    target_link_libraries(${target_name} PUBLIC GTest::gtest GTest::gtest_main GTest::gmock GTest::gmock_main ${libraries})
    gtest_discover_tests(${target_name})
endfunction()

# copy mdx module runtime to the target
function(target_copy_mdx_module_runtime target module)
    if(NOT ${module}_FOUND)
        find_package(${module} REQUIRED)
    endif()
    if(${module}_FOUND)
        foreach(lib ${${module}_LIBRARIES})
            add_custom_command(
                TARGET ${target}
                PRE_BUILD
                # POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:${lib}> $<TARGET_FILE_DIR:${target}>
                VERBATIM
            )
        endforeach()
        foreach(exe ${${module}_EXECUTABLES})
            add_custom_command(
                TARGET ${target}
                PRE_BUILD
                # POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:${exe}> $<TARGET_FILE_DIR:${target}>
                VERBATIM
            )
        endforeach()

        if(${module}_LIBRARY_DIRS) # seen to not work
            target_link_directories(${target} PRIVATE ${${module}_LIBRARY_DIRS})
        endif()
    else()
        message(WARNING "mdx-file: copy runtime fail! ${module} not found! " )
    endif()
endfunction()

# copy mdx module runtime to the target
function(target_copy_mdx_bin_runtime target directory filename)
    add_custom_command(
        TARGET ${target}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${directory}/${filename}" $<TARGET_FILE_DIR:${target}>
        VERBATIM
    )
endfunction()

function(target_copy_shared_lib_dir target directory)
    add_custom_command(
        TARGET ${target}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory copy "${directory}" $<TARGET_FILE_DIR:${target}>
        VERBATIM
    )
endfunction()



