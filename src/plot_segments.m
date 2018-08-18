# plot all segments in workspace

N = length(segments)
i = 1;
while (i < N)
  plot_track3(i, segments, POS, label)
  i = i + 1
endwhile
