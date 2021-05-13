function plot_track_color(segments, data,  label, snum, ...
  origin=[39.8420194 -105.2123333 1808], levelThresh=10)
  
  pkg load quaternion
  pkg load geometry
  
  # default origin is west pilot station
  # pilot station east:  39°50'31.27"N 105°12'44.40"W  39.8420194 -105.2123333
  # pilot station west:  39°50'31.52"N 105°12'45.62"W  39.8420888 -105.2126722

  startTime = segments{snum}(1);

  # if this isn't the last maneuver, include the straight and level exit
  if (snum < length(segments))
    endTime = segments{snum+1}(2);
  else
    endTime = segments{snum}(2) - 30;
  endif

  plot_maneuver_simplified(startTime, endTime, data, snum, label);

endfunction
