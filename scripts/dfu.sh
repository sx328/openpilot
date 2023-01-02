#!/bin/bash

./gpio.sh 124 1  # Reset STM
./gpio.sh 134 $1 # Pull ST_BOOT0 high
sleep 2
./gpio.sh 124 0  # Start STM
./gpio.sh 134 0
