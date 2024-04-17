cmake_minimum_required(VERSION 3.0.2)

project(enio_krizman)

FIND_PACKAGE(Boost)

include_directories(${Boost_INCLUDE_DIR})

add_executable(enio_krizman enio_krizman.cpp)

ADD_DEFINITIONS("-DHAS_BOOST")
