#!/bin/bash

echo $1 > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio$1/direction
echo $2 > /sys/class/gpio/gpio$1/value
