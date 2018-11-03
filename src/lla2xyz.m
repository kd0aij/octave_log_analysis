function xyzr = lla2xyz(pos, theta, origin)
  # pos is an array with columns [lat lon alt]
  # convert pos=[lat lon alt] to [x y z_agl] with rotation theta about z axis
  # convert alt to altitude above ground level by subtracting origin altitude
  # origin is [lat lon alt]
  # lat, lon and theta are in degrees

  # convert to meters from start position
  x = pos(:,2) - origin(:,2);
  y = pos(:,1) - origin(:,1);
  x = x * 111111 * cosd(pos(1));
  y = y * 111111;
  alt = pos(:,3) - origin(:,3);

  # rotate about z axis
  xyzr=[cosd(theta)*x.-sind(theta)*y, sind(theta)*x.+cosd(theta)*y, alt];
  
endfunction
