cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME mc_kortex)
set(PROJECT_DESCRIPTION "Kinova Kortex interface for mc-rtc")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(KORTEX_API_VERSION 2.6.0)

project(${PROJECT_NAME} LANGUAGES C CXX VERSION 1.0.0)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

find_package(mc_rtc REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options thread)

# Download kortex_api if needed
if(UNIX AND NOT APPLE AND NOT EMSCRIPTEN)
  add_definitions(-D_OS_UNIX)
  set(KORTEX_API_URL "https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/${KORTEX_API_VERSION}/linux_x86_64_gcc_5.4.zip")
  file(DOWNLOAD "${KORTEX_API_URL}" "${CMAKE_CURRENT_BINARY_DIR}/kortex_api/kortex_api.zip")
  file(ARCHIVE_EXTRACT
        INPUT "${CMAKE_CURRENT_BINARY_DIR}/kortex_api/kortex_api.zip"
        DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/kortex_api/${KORTEX_API_VERSION}"
  )
  set(KORTEX_ROOT_DIR "${CMAKE_CURRENT_BINARY_DIR}/kortex_api/${KORTEX_API_VERSION}")
else()
  if(NOT DEFINED KORTEX_ROOT_DIR)
    message(FATAL_ERROR "Kortex API must be downloaded and installed manually on your platform, you must then define KORTEX_ROOT_DIR")
  endif()
endif()

# Load kortex_api
link_libraries(${KORTEX_ROOT_DIR}/lib/release/libKortexApiCpp.a)
include_directories(${KORTEX_ROOT_DIR}/include)
include_directories(${KORTEX_ROOT_DIR}/include/client)
include_directories(${KORTEX_ROOT_DIR}/include/common)
include_directories(${KORTEX_ROOT_DIR}/include/messages)
include_directories(${KORTEX_ROOT_DIR}/include/client_stubs)

# link other libs
if(UNIX)
  link_libraries(pthread)
elseif(WIN32)
  link_libraries(winMM ws2_32)
else()
  MESSAGE(FATAL_ERROR "Unknown os! Not supported yet")
endif()

add_subdirectory(src)
