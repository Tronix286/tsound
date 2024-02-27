if (TARGET tinyusb_device)

    add_subdirectory(audio)

    add_definitions(-DPICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1)

    add_executable(tsound
            tsound.c
            ymodem.c
            )

    #add_compile_definitions(PICO_FLASH_SPI_CLKDIV=4)

    pico_generate_pio_header(tsound ${CMAKE_CURRENT_LIST_DIR}/mpi_io.pio)
    # pull in common dependencies
    target_link_libraries(tsound pico_stdlib pico_multicore hardware_pio hardware_dma hardware_spi audio)

    # enable usb output, disable uart output
    pico_enable_stdio_usb(tsound 1)
    pico_enable_stdio_uart(tsound 0)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(tsound)

    # add url via pico_set_program_url
    example_auto_set_url(tsound)

    pico_define_boot_stage2(slower_boot2_tsound ${PICO_DEFAULT_BOOT_STAGE2_FILE})
    target_compile_definitions(slower_boot2_tsound PRIVATE PICO_FLASH_SPI_CLKDIV=4)

    pico_set_boot_stage2(tsound slower_boot2_tsound)


elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()
