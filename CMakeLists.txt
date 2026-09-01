cmake_minimum_required(VERSION 3.21)

set(PROJECT_NAME mc_kortex)
set(PROJECT_DESCRIPTION "Kinova Kortex interface for mc-rtc")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(KORTEX_API_VERSION 2.8.0)

project(
  ${PROJECT_NAME}
  LANGUAGES C CXX
  VERSION 1.0.0)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

find_package(mc_rtc REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options thread)

# Download kortex_api if needed
if(UNIX
   AND NOT APPLE
   AND NOT EMSCRIPTEN)
  add_definitions(-D_OS_UNIX)
  # Kinova renamed the x86_64 archive to x86-64 in API 2.7.0
  if(KORTEX_API_VERSION VERSION_LESS 2.7.0)
    set(KORTEX_API_ARCHIVE "linux_x86_64_gcc_5.4.zip")
  else()
    set(KORTEX_API_ARCHIVE "linux_x86-64_gcc_5.4.zip")
  endif()
  set(KORTEX_API_URL
      "https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/${KORTEX_API_VERSION}/${KORTEX_API_ARCHIVE}"
  )
  set(KORTEX_ROOT_DIR
      "${CMAKE_CURRENT_BINARY_DIR}/kortex_api/${KORTEX_API_VERSION}")
  # Only download once: this runs at configure time, re-fetching the archive on
  # every configure also breaks the build when offline
  if(NOT EXISTS "${KORTEX_ROOT_DIR}/include")
    message(STATUS "Downloading the Kortex API ${KORTEX_API_VERSION}")
    file(
      DOWNLOAD "${KORTEX_API_URL}"
      "${CMAKE_CURRENT_BINARY_DIR}/kortex_api/kortex_api.zip"
      SHOW_PROGRESS
      INACTIVITY_TIMEOUT 60
      TLS_VERIFY ON
      STATUS KORTEX_API_DOWNLOAD_STATUS)
    list(GET KORTEX_API_DOWNLOAD_STATUS 0 KORTEX_API_DOWNLOAD_CODE)
    if(NOT KORTEX_API_DOWNLOAD_CODE EQUAL 0)
      list(GET KORTEX_API_DOWNLOAD_STATUS 1 KORTEX_API_DOWNLOAD_MESSAGE)
      file(REMOVE "${CMAKE_CURRENT_BINARY_DIR}/kortex_api/kortex_api.zip")
      message(
        FATAL_ERROR
          "Failed to download the Kortex API from ${KORTEX_API_URL}: ${KORTEX_API_DOWNLOAD_MESSAGE}"
      )
    endif()
    file(ARCHIVE_EXTRACT INPUT
         "${CMAKE_CURRENT_BINARY_DIR}/kortex_api/kortex_api.zip" DESTINATION
         "${KORTEX_ROOT_DIR}")
  endif()
else()
  if(NOT DEFINED KORTEX_ROOT_DIR)
    message(
      FATAL_ERROR
        "Kortex API must be downloaded and installed manually on your platform, you must then define KORTEX_ROOT_DIR"
    )
  endif()
endif()

# Load kortex_api
link_libraries(${KORTEX_ROOT_DIR}/lib/release/libKortexApiCpp.a)
# SYSTEM: the Kortex and bundled protobuf headers are not warning clean
include_directories(SYSTEM ${KORTEX_ROOT_DIR}/include)
include_directories(SYSTEM ${KORTEX_ROOT_DIR}/include/client)
include_directories(SYSTEM ${KORTEX_ROOT_DIR}/include/common)
include_directories(SYSTEM ${KORTEX_ROOT_DIR}/include/messages)
include_directories(SYSTEM ${KORTEX_ROOT_DIR}/include/client_stubs)

# link other libs
if(UNIX)
  link_libraries(pthread)
elseif(WIN32)
  link_libraries(winMM ws2_32)
else()
  message(FATAL_ERROR "Unknown os! Not supported yet")
endif()

add_subdirectory(src)
