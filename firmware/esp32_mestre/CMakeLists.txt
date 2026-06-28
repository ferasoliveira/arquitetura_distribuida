cmake_minimum_required(VERSION 3.16)

# Permite passar -DEB15_QEMU=1 no build: idf.py build -DEB15_QEMU=1
# A flag eh propagada para o componente main via target_compile_definitions.

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(eb15_mestre VERSION 1.0.0 LANGUAGES C CXX)
