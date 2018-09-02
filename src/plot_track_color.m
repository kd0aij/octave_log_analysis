function plot_track_color(index1, segments, data, ...
  label='', boxCenter=[39.843,-105.2125], levelThresh=15)

startTime = segments{index1}(1);

# if this isn't the last maneuver, include the straight and level exit
if (index1 < length(segments))
  endTime = segments{index1+1}(2);
else
  endTime = segments{index1}(2);
endif

plot_tseg_color2(startTime, endTime, data, index1, label, ...
  boxCenter, levelThresh);

endfunction
