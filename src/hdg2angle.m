function angle = hdg2angle(hdg)
  # convert hdg from [0,360] (positive CW) to 
  # [180,-180] (positive CCW) (with North at 0)
  if hdg > 180
    angle = 360 - hdg;
  else
    angle = -hdg;
  endif
endfunction

