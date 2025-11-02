cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Set the project name
set(CMAKE_PROJECT_NAME car)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

include(cmake/lib.cmake)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_CURRENT_SOURCE_DIR}/core/src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/src/sysmem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/src/syscalls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/src/system_stm32f1xx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32f103xb.s

    ${CMAKE_CURRENT_SOURCE_DIR}/core/src/state.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/src/comm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/src/err.c
    ${CMAKE_CURRENT_SOURCE_DIR}/core/src/IRQHandler.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/drv
    ${CMAKE_CURRENT_SOURCE_DIR}/core/inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    # Add user defined libraries
    m
    lib
)

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)