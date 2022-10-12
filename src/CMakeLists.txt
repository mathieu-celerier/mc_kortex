set(mc_kortex_lib_SRC
    mc_kortex.cpp
    KinovaControlLoop.cpp
    KinovaRobot.cpp
)

set(mc_kortex_lib_HDR
    mc_kortex.h
    KinovaControlLoop.h
    KinovaRobot.h
)

add_library(mc_kortex_lib STATIC ${mc_kortex_lib_SRC} ${mc_kortex_lib_HDR})
target_link_libraries(mc_kortex_lib PUBLIC mc_rtc::mc_control)

add_executable(mc_kortex main.cpp)
target_link_libraries(mc_kortex PRIVATE mc_kortex_lib Boost::program_options)

install(TARGETS mc_kortex)