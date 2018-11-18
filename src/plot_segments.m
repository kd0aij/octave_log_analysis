# plot all segments in workspace

N = length(segs)
i = 1;
while (i < N)
  plot_track_color(segs, data, label, i)
##  pause(5);
  close all
  i = i + 1
endwhile
