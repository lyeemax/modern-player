# Default installation directory, based on operating system
include(GNUInstallDirs)

IF (PLAYER_OS_WIN)
    SET (CMAKE_INSTALL_PREFIX "C:\\Program Files\\Player" CACHE PATH "Installation prefix")
ELSE (PLAYER_OS_WIN)
    SET (CMAKE_INSTALL_PREFIX "/usr/local" CACHE PATH "Installation prefix")
ENDIF (PLAYER_OS_WIN)

MESSAGE (STATUS "Player will be installed to ${CMAKE_INSTALL_PREFIX}")

# Installation prefix for include files
STRING (TOLOWER ${PROJECT_NAME} projectNameLower)
SET (PLAYER_INCLUDE_INSTALL_DIR "${CMAKE_INSTALL_INCLUDEDIR}/${projectNameLower}-${PLAYER_MAJOR_VERSION}.${PLAYER_MINOR_VERSION}")

# Let the user take care of this
SET (PLAYER_LIBRARY_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})
SET (PLAYER_BINARY_INSTALL_DIR ${CMAKE_INSTALL_BINDIR})
SET (PLAYER_DATA_INSTALL_DIR ${CMAKE_INSTALL_DATADIR})
SET (PLAYER_PLUGIN_INSTALL_DIR "${PLAYER_LIBRARY_INSTALL_DIR}/${projectNameLower}-${PLAYER_MAJOR_VERSION}.${PLAYER_MINOR_VERSION}")

MESSAGE (STATUS "Headers will be installed to ${CMAKE_INSTALL_PREFIX}/${PLAYER_INCLUDE_INSTALL_DIR}")
MESSAGE (STATUS "Libraries will be installed to ${CMAKE_INSTALL_PREFIX}/${PLAYER_LIBRARY_INSTALL_DIR}")
MESSAGE (STATUS "Plugins will be installed to ${CMAKE_INSTALL_PREFIX}/${PLAYER_PLUGIN_INSTALL_DIR}")