#!/bin/bash

echo "generating matlab/octave m-file"
mavtomfile.py --types POS,GPS,ATT,IMU,NKF1 "$1".bin

