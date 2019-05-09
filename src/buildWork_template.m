# parse dataflash logfile using dflog2octave.sh which invokes
# "mavtomfile.py --types POS,GPS,ATT,IMU,NKF1 logfile.bin"

# run the resulting mfile
run temp.m

# generate a workspace containing the desired data
data = alignData(POS, GPS, ATT, IMU, NKF1, NKQ1);
segments = segment_maneuvers2(2, 0, data);

# and save to temp.work
save 'temp.work'
