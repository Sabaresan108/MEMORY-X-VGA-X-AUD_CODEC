SRAM_WR and SRAM_RD are common for all the top modules present in this repository except SDRAM because it's a seprate moodule related to SDRAM and AUD_CODEC.

The Audio_loopback code is a seperate code which doesn't have any connection with SRAM.

The SRAM_VGA dosen't use SRAM_WR and SRAM_RD because it is an independent code.

SDRAM uses the AUDIO_CODEC code for it's input structure.
