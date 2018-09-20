function xyzr = lla2xyz(pos, theta, origin)
  # convert pos=[lat lon alt] to [x y z agl] with rotation theta about z axis
  # convert alt to altitude above ground level by subtracting gndAlt
  # origin is [lat lon alt]

  # convert to meters from start position
  x = pos(:,2) - origin(:,2);
  y = pos(:,1) - origin(:,1);
  x = x * 111111 * cos(pos(1)*pi/180);
  y = y * 111111;
  alt = pos(:,3) - origin(:,3);

  # rotate about z axis
  xyzr=[cos(theta)*x.-sin(theta)*y, sin(theta)*x.+cos(theta)*y, alt];
  
endfunction
