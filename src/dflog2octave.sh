#!/bin/bash
#arg 1 is basepath
#arg 2 is logfile name (no extension)

echo "basepath:" $1
echo "input logfile:" $2
cd $1
echo "running mavtomfile.py --types POS,GPS,ATT,IMU,NKF1,NKQ1 "$2".bin"
mavtomfile.py --types POS,GPS,ATT,IMU,NKF1,NKQ1 "$2".bin
echo "renaming "$2".m to temp.m"
mv $2".m" ~/sf_markw/git/kd0aij/octave_log_analysis/src/temp.m
cd ~/sf_markw/git/kd0aij/octave_log_analysis/src
echo "running octave --no-gui buildWorkspace.m"
octave --no-gui buildWorkspace.m
echo "saving workspace as "$2".work"
mv "temp.work" $2".work"
