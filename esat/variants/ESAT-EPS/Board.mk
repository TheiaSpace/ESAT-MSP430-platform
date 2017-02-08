######################################
# Board specific
ARCH = msp430
MCU = msp430f5529
MCU_FLAG = -mmcu=$(MCU)
F_CPU = 25000000L
FLASH_SIZE = 131072
UPLOAD_COMMAND = msp430-tool bsl5.hid -e build/$(SKETCH_NAME).cpp.hex
#UPLOAD_COMMAND = $(MSPDEBUG) $(VERBOSE_UPLOAD) load-bsl --force-reset "prog build/$(SKETCH_NAME).cpp.hex"
#UPLOAD_COMMAND = $(MSPDEBUG) $(VERBOSE_UPLOAD) tilib --force-reset "prog build/$(SKETCH_NAME).bin"
######################################
