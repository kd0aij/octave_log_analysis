function [roll pitch r2hzp] = maneuver_roll_pitch(rhdg, quat)
  pkg load geometry;
  pkg load quaternion;
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

##  # first rotate body x around heading vector till it's parallel with
##  # hzplane (perpendicular to plane normal hzplane)
##  # project bx into plane normal to hzplane and rotate in this plane
##  bparhv = cross(hv, (cross(bx, hv)));
####  bprphv = dot(bx, hv);
##  
##  # (R(hv,theta) * bx) cross hzplane = 0
##  theta = acos(vectorNorm(bparhv));  
##  r2hzp = rot2q(hv, theta);
##  
##  by = hamilton_product(quat, [0 1 0]);

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
  bxchz = cross(hamilton_product(fq, [1 0 0]), hzplane);
  
  # calculate Euler pitch
  pitch = real(rad2deg( asin(2*(fq.w*fq.y - fq.z*fq.x))));

  # roll is zero when plane of wings is perpendicular to maneuver plane
  
  # perpendicular to body x-z plane transformed to earth frame
  xyplane = hamilton_product(fq, [0 1 0]);
  
  # angle between wing plane and maneuver plane
  # This isn't always the x component
  xy_cross_hz = cross(hzplane, xyplane);
  # try this to get the sign right
  xy_cross_hz = sign(sum(xy_cross_hz)) * vectorNorm(xy_cross_hz);
  
  # angle between wing plane and maneuver plane
  xydothz = dot(xyplane, hzplane);
  
  # this gives a range of [-180, 180] 
  roll = -atan2d(xy_cross_hz, xydothz);
endfunction

