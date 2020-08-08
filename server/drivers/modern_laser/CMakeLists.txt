
CHECK_INCLUDE_FILES (linux/serial.h HAVE_LINUX_SERIAL_H)
SET (laser_config_h_in ${CMAKE_CURRENT_SOURCE_DIR}/laser_config.h.in)
SET (laser_config_h ${CMAKE_CURRENT_BINARY_DIR}/laser_config.h)
CONFIGURE_FILE (${CMAKE_CURRENT_SOURCE_DIR}/laser_config.h.in ${CMAKE_CURRENT_BINARY_DIR}/laser_config.h)

FILE(GLOB RPLIDAR_SDK_SRC
        "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
        "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
        "${RPLIDAR_SDK_PATH}/src/*.cpp"
        )
PLAYERDRIVER_OPTION (Rplidar build_Rplidar ON)
PLAYERDRIVER_REJECT_OS (Rplidar build_Rplidar PLAYER_OS_WIN)
PLAYERDRIVER_ADD_DRIVER (Rplidar build_Rplidar SOURCES Rplidar.cc ${RPLIDAR_SDK_SRC})

