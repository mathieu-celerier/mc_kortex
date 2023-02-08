cmake_minimum_required(VERSION 3.0)

set(PROJECT_NAME mc_kortex)
set(PROJECT_DESCRIPTION "Kinova Kortex interface for mc-rtc")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

project(${PROJECT_NAME} LANGUAGES C CXX VERSION 1.0.0)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

find_package(mc_rtc REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options thread)

set(KORTEX_DIR "${PROJECT_SOURCE_DIR}/extern/kortex_api")
### KORTEX RELATED
if(UNIX)
  add_definitions(-D_OS_UNIX)
elseif(WIN32)
  add_definitions(-D_OS_WINDOWS -DNOMINMAX)
  if(MSVC)
     add_compile_options(/bigobj)
  endif()
endif()

# Load kortex_api
link_libraries(${KORTEX_DIR}/lib/release/libKortexApiCpp.a)
include_directories(${KORTEX_DIR}/include)
include_directories(${KORTEX_DIR}/include/client)
include_directories(${KORTEX_DIR}/include/common)
include_directories(${KORTEX_DIR}/include/messages)
include_directories(${KORTEX_DIR}/include/client_stubs)

# link other libs
if(UNIX)
  link_libraries(pthread)
elseif(WIN32)
  link_libraries(winMM ws2_32)
else()
  MESSAGE(FATAL_ERROR "Unknown os! Not supported yet")
endif()

add_subdirectory(src)
