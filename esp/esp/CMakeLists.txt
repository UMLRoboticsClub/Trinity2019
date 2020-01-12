# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)
project(esp)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(EXTRA_COMPONENT_DIRS "components")
set(COMPONENT_ADD_INCLUDEDIRS, ${components/rosserial_esp32/include})

