function [roll pitch wca wca_axis reverse] = maneuver_roll_pitch(rhdg, quat)
  # given the maneuver heading: rhdg
  # calculate roll angle as angle between rhdg/earthz plane and body x/y plane
  # input quat is an array [w x y z] or quaternion object: q.w q.i q.j q.k
  if not(isa(quat,'quaternion') )
    q = quat;
    quat = quaternion(q(1),q(2),q(3),q(4));
  endif
  bx = hamilton_product(quat, [1 0 0]);
##  bz = hamilton_product(quat, [0 0 1]);

  # hzplane is the normal vector which defines the maneuver plane 
  # this hzplane requires maneuvers to lie in a vertical plane parallel to rhdg
  hzplane = [-sind(rhdg) cosd(rhdg) 0];
  
##  bzdhz = dot(bz, hzplane);
  
  # a more general version would allow the maneuver plane to be non-vertical
  # where mplane is (hv cross earthz) rotated about hv by a roll angle
##  hv = [cosd(rhdg) sind(rhdg) 0];
##  hzplane = cross(hv, mplane);

  # the wind correction angle (WCA) relative to flight path is the
  # angle between body frame x and hzplane
  # This should be independent of roll and pitch: roll does not affect the direction
  # of bx and pitch is a rotation about hzplane, which does not change the angle
  wca_axis = cross(bx, hzplane);
  wca = 90 - atan2d(vectorNorm(wca_axis), dot(bx,hzplane));
  
  # to back out wca, rotate about cross(bx, hzplane) 
  wca_axis = wca_axis / vectorNorm(wca_axis);
  # this will be inv(quatc) if correct
  r2hzp = rot2q(wca_axis, deg2rad(real(-wca)));
  # this is the attitude with body x rotated into maneuver plane
  fq = unit(r2hzp * quat);
  
  # calculate Euler pitch in maneuver plane
  [eroll pitch eyaw] = quat2euler(fq);

  # HACK: reverse rhdg if sign of euler yaw is different from that of rhdg
  # this is detecting a reversal in ground course at low gspd, but I had thought
  # that flipping the hzplane normal shouldn't affect the results
  reverse = false;
  if sign(eyaw) != sign(rhdg)
      reverse = true;
      rhdg = wrap180(rhdg + 180);
  end
  
  # back out rhdg and pitch
  ryaw = rot2q([0 0 1], deg2rad(-rhdg));
  rpitch = rot2q([0 1 0], deg2rad(-pitch));
  
  # remaining rotation should be roll relative to maneuver plane
  rollq = unit(rpitch * ryaw * fq);
  [axisr, thetar] = q2rot(rollq);
  direction = dot(axisr, [1 0 0]);
  roll = sign(direction) * wrap(rad2deg(thetar));
endfunction

