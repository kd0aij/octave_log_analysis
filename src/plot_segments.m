# plot all segments in workspace

N = length(segments)
i = 1;
while (i < N)
  plot_track_color(i, segments, data, label)
  pause(5);
  close all
  i = i + 1
endwhile
