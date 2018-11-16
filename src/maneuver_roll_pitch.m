function [roll pitch wca wca_axis] = maneuver_roll_pitch(rhdg, quat, pThresh)
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
  bz = hamilton_product(quat, [0 0 1]);

  # a more general version would allow the maneuver plane to be non-vertical
  # where mplane is (hv cross earthz) rotated about hv by a roll angle
##  hzplane = cross(hv, mplane);  

  # the wind correction angle (WCA) relative to flight path is the
  # angle between body frame x and hzplane
  # This should be independent of roll and pitch: roll does not affect the direction
  # of bx and pitch is a rotation about hzplane, which does not change the angle
  wca_axis = cross(bx, hzplane);
  wca = 90 - asind(vectorNorm(wca_axis));
  
  # to back out wca, rotate about cross(bx, hzplane) 
  wca_axis = wca_axis / vectorNorm(wca_axis);
  # this will be inv(quatc) if correct
  r2hzp = rot2q(wca_axis, deg2rad(real(-wca)));
  # this is the attitude with body x rotated into maneuver plane
  fq = unit(r2hzp * quat);
  
  # calculate Euler pitch in maneuver plane
  pitch = real(rad2deg( asin(2*(fq.w*fq.y - fq.z*fq.x))));
  rolle  = rad2deg(atan2(2*(fq.w*fq.x + fq.y*fq.z), 1 - 2*(fq.x*fq.x + fq.y*fq.y)));
  [roll pitch yaw] = quat2euler(fq);
##  if abs(pitch) < pThresh
##    return 
##  endif
  
  # back out rhdg and pitch
  ryaw = rot2q([0 0 1], deg2rad(-rhdg));
  rpitch = rot2q([0 1 0], deg2rad(-pitch));
  
  # remaining rotation should be roll relative to maneuver plane
  rollq = unit(rpitch * ryaw * fq);
  [axisr, thetar] = q2rot(rollq);
  direction = dot(axisr, [1 0 0]);
  roll = sign(direction) * wrap(rad2deg(thetar));
endfunction

