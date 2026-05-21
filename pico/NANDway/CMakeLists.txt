# NANDway Signal Booster Build
add_executable(NANDway_SBE
    main.c
)

target_compile_definitions(NANDway_SBE PRIVATE BUILD_VERSION=2)
target_include_directories(NANDway_SBE PUBLIC ../common)
target_link_directories(NANDway_SBE PUBLIC ../build/common)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(NANDway_SBE pico_stdlib)
target_link_libraries(NANDway_SBE flashway_common_${PICO_PLATFORM})

# Setup SDK features
pico_enable_stdio_usb(NANDway_SBE 1)
pico_enable_stdio_uart(NANDway_SBE 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(NANDway_SBE)
