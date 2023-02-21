cmake_minimum_required(VERSION 3.7)

project(8085)

if(MSVC)
  add_compile_options(/MP) # Use multiple processors when building
  add_compile_options(/W4 /wd4201 /WX) # Warning level 4, all warnings are
                                       # errors
else()
  add_compile_options(-W -Wall) # All Warnings, all warnings are errors
endif()

set (CMAKE_CXX_STANDARD 20)

set(EMULATOR_SOURCES "cpu.h" "cpu.cpp")

source_group("src" FILES ${EMULATOR_SOURCES})

add_library(8085 ${EMULATOR_SOURCES})

target_include_directories (8085 PUBLIC "${PROJECT_SOURCE_DIR}")
