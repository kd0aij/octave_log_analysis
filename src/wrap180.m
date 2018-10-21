function deg = wrap180(deg)
  # wrap to [-180,180]
  while deg > 180
    deg -= 360;
  endwhile
  while deg < -180
    deg += 360;
  endwhile
endfunction
