#!/bin/bash
#set -x #echo on

DST_DIR="/home/pi/esp/build"

DST_USER="pi"
DST_HOST="192.168.1.5"
PUB_KEY="/home/jackson/.ssh/id_rsa_robotpi"

send_file() { 
    SRC_FILE=$1
    DST_FILE="$(basename "${SRC_FILE}")"
    scp -i "$PUB_KEY" "$SRC_FILE" "$DST_USER"@"$DST_HOST":"$DST_DIR"
}

#BOOTLOADER="/home/jackson/esp/motorcontrol/build/bootloader/bootloader.bin"
#PROJECT="/home/jackson/esp/motorcontrol/build/blink.bin"
#PARTITIONS="/home/jackson/esp/motorcontrol/build/partitions_singleapp.bin"

PROJECT="/tmp/mkESP/main_esp32/main.bin"
PARTITIONS="/tmp/mkESP/main_esp32/main.partitions.bin"

#send_file $PROJECT
#send_file $PARTITIONS

DST_PROJECT="$DST_DIR/$(basename "${PROJECT}")"
DST_PARTITIONS="$DST_DIR/$(basename "${PARTITIONS}")"

DST_PORT="/dev/ttyUSB0"

CMD="esptool --chip esp32 --port $DST_PORT --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x10000 $DST_PROJECT 0x8000 $DST_PARTITIONS"
#CMD="python esptool.py --chip esp32 --port /dev/ttyAMA0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 $BOOTLOADER 0x10000 $PROJECT 0x8000 $PARTITIONS"

ssh -i "$PUB_KEY" "$DST_USER"@"$DST_HOST" "$CMD"
