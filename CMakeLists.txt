cmake_minimum_required(VERSION 2.4.7)
PROJECT(indi-jolohub CXX C)

set (VERSION_MAJOR 0)
set (VERSION_MINOR 2)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
set(BIN_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/bin")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")


find_package(INDI REQUIRED)

include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${INDI_INCLUDE_DIR})

################ JoloHub ################

set(indi_jolohub_SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/indi_jolohub.cpp
   )

add_executable(indi_jolohub ${indi_jolohub_SRCS})
target_link_libraries(indi_jolohub indidriver)
install(TARGETS indi_jolohub RUNTIME DESTINATION bin )
install(FILES indi_jolohub.xml DESTINATION ${INDI_DATA_DIR})

