zephyr_library()

zephyr_library_sources(board.c)

if(CONFIG_PPOD_V2P0_UTILS)
    zephyr_library_sources(ppod_v2p0_utils.c)
endif()
