function [roll, pitch, yaw] = attFromQuat(q, avgYaw, pthresh)
  # input q is an array [w x y z] 
  # q = w + xi + yj + zk
  # result is xyz-fixed Euler angles in degrees
  # this rotation is from body to earth-fixed (NED)
  [roll pitch yaw] = quat2euler(q);
  
  # avoid the singularity at abs(pitch)==90
  # for pattern judging, we need roll w.r.t. the maneuver heading, which
  # is input as avgYaw. But since it's the trajectory which must be vertical
  # on up/down lines, pitch and yaw will be dependent on wind, not necessarily
  # +/-90 and avgYaw, respectively...
  # Assume that pitch angles > pthresh degrees are verticals, and force
  # yaw to avgYaw
  if abs(pitch) > pthresh
    quat = quaternion(q(1),q(2),q(3),q(4));
    noseup = sign(pitch);
    
    # compute roll angle by projecting body-frame Y axis to earth X-Y plane
    # second column is Y axis w.r.t. Earth frame
    [axis, theta] = q2rot(unit(quat));
    rmat = rotv(axis', theta);
    rmat = rmat';
    y_x = rmat(1,2);
    y_y = rmat(2,2);
    roll = -rad2deg(atan2(y_x, y_y)) - avgYaw;
    roll = hdg2angle(roll);
    yaw = avgYaw;
##    printf("2) roll: %5.3f, pitch: %5.3f, yaw: %5.3f\n", roll, pitch, yaw);
  endif
endfunction
