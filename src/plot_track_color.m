function plot_track_color(index1, segments, data, ...
  label='', origin=[39.8420194 -105.2123333 1808], levelThresh=15, posIndex=2)
  
# default origin is west pilot station
# pilot station east:  39°50'31.27"N 105°12'44.40"W  39.8420194 -105.2123333
# pilot station west:  39°50'31.52"N 105°12'45.62"W  39.8420888 -105.2126722

startTime = segments{index1}(1);

# if this isn't the last maneuver, include the straight and level exit
if (index1 < length(segments))
  endTime = segments{index1+1}(2);
else
  endTime = segments{index1}(2) - 30;
endif

plot_tseg_color2(startTime, endTime, data, index1, label, ...
  origin, levelThresh);

endfunction
