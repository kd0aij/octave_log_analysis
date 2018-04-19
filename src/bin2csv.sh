#!/bin/bash

echo "generating: " "$1"_att.csv
mavlogdump.py --format csv --types ATT $1.bin > "$1"_att.csv

echo "generating: " "$1"_gps.csv
mavlogdump.py --format csv --types GPS $1.bin > "$1"_gps.csv

echo "generating: " "$1"_imu.csv
mavlogdump.py --format csv --types IMU $1.bin > "$1"_imu.csv

echo "generating: " "$1"_nkf1.csv
mavlogdump.py --format csv --types NKF1 $1.bin > "$1"_nkf1.csv

