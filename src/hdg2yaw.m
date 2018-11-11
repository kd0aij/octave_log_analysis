function hdg = hdg2yaw(hdg)
  # convert hdg from [0,360] to 
  # [180,-180] (with North at 0)
##  if hdg > 180
##    angle = hdg - 360;
##  else
##    angle = hdg;
##  endif
  hdg(hdg>180) -= 360;
endfunction


