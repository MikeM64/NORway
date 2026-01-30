cmake_minimum_required(VERSION 3.13)

# initialize pico-sdk from GIT
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_FETCH_FROM_GIT on)

# pico_sdk_import.cmake is a single file copied from this SDK
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(NORway_rp2350b)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project
add_executable(NORway_rp2350b
    main.c
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(NORway_rp2350b pico_stdlib)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(NORway_rp2350b)

