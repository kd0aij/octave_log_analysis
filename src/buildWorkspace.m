# function to process ArduPilot binary logfile 
function buildWorkspace(logfile, sl_dur, p_lp)

# first parse dataflash logfile using dflog2octave.bat <logfile> (no extension)
# which invokes
# "mavtomfile.py --types POS,GPS,ATT,IMU,NKF1 -o <logfile>.m logfile.bin"
# and generates an octave script named <logfile>.m

# run the resulting mfile
disp(sprintf("running file: %s", [logfile '.m']));
eval(['run ' logfile '.m'])

# generate a workspace containing the desired data
data = alignData(POS, GPS, ATT, IMU, NKF1, NKQ1);
segments = segment_maneuvers2(sl_dur, p_lp, data);

# and save to <logfile>.work
disp(sprintf("saving workspace to: %s", [logfile '.work']));
eval(['save ' logfile '.work'])
