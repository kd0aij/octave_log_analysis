# plot all segments in workspace

N = length(segments)
i = 1;
while (i < N)
  plot_track_color(i, segments, POS, ATT, IMU, label)
  pause(5);
  close all
  i = i + 1
endwhile
