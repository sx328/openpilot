#!/bin/bash -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
cd $DIR

./dfu.sh 1
./spi_dfu_flash_h7.py /data/openpilot/panda/board/obj/bootstub.panda_h7.bin /data/openpilot/panda/board/obj/panda_h7.bin.signed
./dfu.sh 0
