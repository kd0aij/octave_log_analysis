function plot_track_color(index1, segments, POS, ATT, ...
  label='', boxCenter=[39.843,-105.2125], levelThresh=15)

# it appears that GPS location is NOT accurate when POS data starts
# skip the first 30 seconds of data
pts = POS.data(:,1);
iniTime = pts(1) + 30;

# vehicle has just departed "straight and level"
startTime = segments{index1}(1);

if (startTime < iniTime) 
  startTime = iniTime;
endif

# if this isn't the last maneuver, include the straight and level exit
if (index1 < length(segments))
  endTime = segments{index1+1}(2);
else
  endTime = segments{index1}(2);
endif

plot_tseg_color(startTime, endTime, POS, ATT, index1, label, boxCenter, levelThresh);

endfunction
