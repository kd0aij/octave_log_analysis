#!/bin/bash
#arg 1 is basepath
#arg 2 is logfile name (no extension)

echo "generating matlab/octave m-file"
echo "basepath:" $1
cd $1
mavtomfile.py --types POS,GPS,ATT,IMU,NKF1 "$2".bin
echo "renaming m_"$2".m to temp.m"
mv "m_"$2".m" ~/sf_markw/git/kd0aij/octave_log_analysis/src/temp.m
cd ~/sf_markw/git/kd0aij/octave_log_analysis/src
octave --no-gui doSegment.m