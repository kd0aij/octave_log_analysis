#!/bin/bash
#arg 1 is basepath
#arg 2 is logfile name (no extension)

echo "basepath:" $1
echo "input logfile:" $2
rm temp.m
cd $1
echo "running mavtomfile.py --types POS,GPS,ATT,IMU,NKF1,NKQ1 "$2".bin"
mavtomfile.py --types POS,GPS,ATT,IMU,NKF1,NKQ1 "$2".bin 
cp "$2".m ~/git/kd0aij/octave_log_analysis/src/temp.m
cd ~/git/kd0aij/octave_log_analysis/src
cat buildWork_template.m > buildWorkspace_tmp.m
echo "label='"$2"'" >> buildWorkspace_tmp.m
echo "label=[label(regexp(label,'[^0]')(1):end)]" >> buildWorkspace_tmp.m
echo "plot_segments" >> buildWorkspace_tmp.m
echo "running flatpak run org.octave.Octave --no-gui buildWorkspace_tmp.m"
flatpak run org.octave.Octave --no-gui buildWorkspace_tmp.m
echo "saving buildWorkspace_tmp.m as buildWorkspace"$2".m"
mv buildWorkspace_tmp.m buildWorkspace"$2".m
echo "saving workspace as "$2".work"
mv "temp.work" $2".work"
echo "moving "$2"*.jpg to "$1"/"
