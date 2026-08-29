cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    components/dw3000-decadriver-source/platform/esp-idf/decadriver
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32s3_uwb_v2v_gateway)
