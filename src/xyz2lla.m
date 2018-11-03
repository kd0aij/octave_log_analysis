function lla = xyz2lla(xyz, theta, origin)
  # convert xyz=[x y z_agl] to [lat lon alt] with rotation theta about z axis
  # convert agl to altitude by adding origin altitude
  # origin is [lat lon alt]
  # lat, lon and theta are in degrees
  
  # rotate about z axis
  xyzr=[cosd(theta)*xyz(:,1).-sind(theta)*xyz(:,2), sind(theta)*xyz(:,1).+cosd(theta)*xyz(:,2), xyz(:,3)];
  
  # convert to degrees from origin
  lng = m2dLng(xyzr(:,1), m2dLat(xyzr(:,2)) + origin(1));
  lat = m2dLat(xyzr(:,2));
  
  # offset to origin
  lng += origin(2);
  lat += origin(1);
  alt = xyzr(:,3) + origin(:,3);
  
  lla = [lat lng alt];

endfunction
