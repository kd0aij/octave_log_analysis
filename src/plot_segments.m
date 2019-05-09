# plot all segments in workspace

N = length(segments)
i = 1;
while (i < N)
  plot_track_color(segments, data, label, i)
##  pause(5);
  close all
  i = i + 1
endwhile
