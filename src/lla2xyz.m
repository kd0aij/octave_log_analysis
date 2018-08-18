function xyzr = lla2xyz(POS, a, gndAlt)
  # convert lat/lon/alt to xyz with rotation a about z axis
  # convert z to altitude above ground level by subtracting refAlt

  lat=POS.data(:,3);
  lon=POS.data(:,4);
  z = POS.data(:,5);

  # convert to meters from start position
  x=lon-POS.data(1,4);
  y=lat-POS.data(1,3);
  x=x*53*5280/3.28;
  y=y*69*5280/3.28;
  z -= gndAlt;

  # rotate about z axis
  xyzr=[cos(a)*x.-sin(a)*y, sin(a)*x.+cos(a)*y, z];
  
endfunction
