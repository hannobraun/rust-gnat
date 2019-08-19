#!/usr/bin/env bash
set -e

# Flashes a binary built by Cargo to the Tlera Corp Gnat board, using the STM32
# bootloader.
#
# Requires dfu-util:
# http://dfu-util.sourceforge.net/
#
# This will only work if the bootloader is active. To start the bootloader, do
# the following:
# 1. Press the BTN button on the Gnat and keep it pressed.
# 2. Press and release the RST button.
# 3. Relase the BTN button.
#
# After this procedure, the bootloader should be running, and dfu-util should be
# able to download the program.

ELF_FILE=$1
BIN_FILE=$ELF_FILE.bin

cargo objcopy -- --input-target=elf --output-target=binary $ELF_FILE $BIN_FILE

dfu-util \
    --device 0483:df11 \
    --alt 0 \
    --dfuse-address 0x08000000:leave \
    --download $BIN_FILE
