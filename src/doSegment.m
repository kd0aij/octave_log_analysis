#function retval = doSegment(logfile)

# parse dataflash logfile using "mavtomfile.py --types POS,GPS,ATT,IMU,NKF1 logfile.bin"
# run the resulting mfile "run m_logfile.m"
#mfile = ['m_' logfile '.m'];
run temp.m
segments = segment_maneuvers(3.0, 0.95, ATT, GPS, POS)

# and save to temp.work
save 'temp.work'

plot_track(1, 2, segments, POS)
