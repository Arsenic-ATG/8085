cmake_minimum_required(VERSION 3.7)

project(emulator8085)

set (CMAKE_CXX_STANDARD 20)

if(MSVC)
  add_compile_options(/MP) # Use multiple processors when building
  add_compile_options(/W4 /wd4201 /WX) # Warning level 4, all warnings are
                                       # errors
else()
  add_compile_options(-W -Wall) # All Warnings
endif()

# Download and unpack googletest at configure time
configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
execute_process(
  COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download)
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(
  COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download)
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker settings on Windows
set(gtest_force_shared_crt
    ON
    CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines the gtest and gtest_main
# targets.
add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
                 ${CMAKE_CURRENT_BINARY_DIR}/googletest-build EXCLUDE_FROM_ALL)

# source for the test executable
set(emulator_SOURCES
  "src/main_8085.cpp"
  "src/basic_test.cpp")

source_group("src" FILES ${emulator_SOURCES})

add_executable(emulatorTest ${emulator_SOURCES})
add_dependencies(emulatorTest lib8085)
target_link_libraries(emulatorTest gtest)
target_link_libraries(emulatorTest lib8085)
