cmake_minimum_required(VERSION 3.7)

project(cpu)

if(MSVC)
  add_compile_options(/MP) # Use multiple processors when building
  add_compile_options(/W4 /wd4201 /WX) # Warning level 4, all warnings are
                                       # errors
else()
  add_compile_options(-W -Wall) # All Warnings, all warnings are errors
endif()

set (CMAKE_CXX_STANDARD 20)

set(EMULATOR_SOURCES "cpu.h" "cpu.cpp" "main.cpp")

source_group("src" FILES ${EMULATOR_SOURCES})

add_library(cpu ${EMULATOR_SOURCES})

target_include_directories (cpu PUBLIC "${PROJECT_SOURCE_DIR}")
