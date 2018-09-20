function hdg = angle2hdg(angle)
  # convert angle from [180,-180] (positive CCW) to 
  # [0,360] (positive CW) (with North at 0)
  if angle > 0
    hdg = 360 - angle;
  else
    hdg = -angle;
  endif
endfunction

