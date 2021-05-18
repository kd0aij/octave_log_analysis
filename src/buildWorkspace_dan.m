# parse dataflash logfile using dflog2octave.sh which invokes
# "mavtomfile.py --types POS,GPS,ATT,IMU,NKF1 logfile.bin"
function buildWorkspace(label, sl_dur, p_lp)
# run the resulting mfile
run([label ".m"])

# generate a workspace containing the desired data
data = alignData(POS, GPS, ATT, IMU, NKF1, NKQ1);
segments = segment_maneuvers2(2, 0, data);

# and save workspace
save([label '.work'])
##label=''
##label=[label(regexp(label,'[^0]')(1):end)]
##plot_segments

endfunction