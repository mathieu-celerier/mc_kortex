cmake_minimum_required(VERSION 3.10)

project(
    mc_kortex
    VERSION 0.1.0
    DESCRIPTION "mc_rtc interface for kortex api"
    LANGUAGES CXX
)

find_package(mc_rtc REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options thread)
find_package(mc_kinova REQUIRED)

add_subdirectory(src)