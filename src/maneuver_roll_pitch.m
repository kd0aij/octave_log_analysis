function [roll pitch wca wca_axis] = maneuver_roll_pitch(rhdg, quat)
  # input q is an array [w x y z] or quaternion object: q.w q.i q.j q.k
  if not(isa(quat,'quaternion') )
    q = quat;
    quat = quaternion(q(1),q(2),q(3),q(4));
  endif
  # given the maneuver heading: rhdg
  # calculate roll angle as angle between rhdg/earthz plane and body x/y plane
  hv = [cosd(rhdg) sind(rhdg) 0];
  
  # this hzplane requires maneuvers to lie in a vertical plane
  hzplane = [-sind(rhdg) cosd(rhdg) 0];
  
  bx = hamilton_product(quat, [1 0 0]);

  # a more general version would allow the maneuver plane to be non-vertical
  # where mplane is (hv cross earthz) rotated about hv by a roll angle
##  hzplane = cross(hv, mplane);  

  # the wind correction angle (WCA) relative to flight path is the
  # angle between body frame x and hzplane
  # This should be independent of roll and pitch: roll does not affect the direction
  # of bx and pitch is a rotation about hzplane, which does not change the angle
  wca_axis = cross(bx, hzplane);
  wca = 90 - asind(vectorNorm(wca_axis));
  
  # to back out wca, rotate about axis cross(bx, hzplane) 
  wca_axis = wca_axis / vectorNorm(wca_axis);
  # this will be inv(quatc) if correct
  r2hzp = rot2q(wca_axis, deg2rad(real(wca)));
  # this is the attitude with body x aligned to maneuver heading
  fq = unit(r2hzp * quat);
##  bxchz = cross(hamilton_product(fq, [1 0 0]), hzplane);
  
  # calculate Euler pitch
  pitch = real(rad2deg( asin(2*(fq.w*fq.y - fq.z*fq.x))));

  # roll is zero when plane of wings is perpendicular to maneuver plane
  
  # perpendicular to body x-z plane transformed to earth frame
  xzplane = hamilton_product(fq, [0 1 0]);
  
  # sine of angle between xzplane and maneuver plane (always positive: theta=[0,180]
  xy_cross_hz = cross(hzplane, xzplane);
  stheta = vectorNorm(xy_cross_hz);
  
  # cosine of angle between wing plane and maneuver plane
  ctheta = dot(xzplane, hzplane);
  
  if dot(xzplane, hv) > 0
    # this gives a range of [0, 180] 
    roll = atan2d(stheta, ctheta);
  else
    roll = -atan2d(stheta, ctheta);
  endif
endfunction

