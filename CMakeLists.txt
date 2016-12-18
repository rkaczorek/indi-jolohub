cmake_minimum_required(VERSION 2.4.7)
PROJECT(indi-astrohub CXX C)

set (VERSION_MAJOR 1)
set (VERSION_MINOR 0)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
set(BIN_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/bin")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")


find_package(INDI REQUIRED)

include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${INDI_INCLUDE_DIR})

################ PiFace Board ################

set(indi_astrohub_SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/indi_astrohub.cpp
   )

add_executable(indi_astrohub ${indi_astrohub_SRCS})
target_link_libraries(indi_astrohub indidriver)
install(TARGETS indi_astrohub RUNTIME DESTINATION bin )
install(FILES indi_astrohub.xml DESTINATION ${INDI_DATA_DIR})

