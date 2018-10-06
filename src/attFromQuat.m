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
  # When vertical, both roll and yaw act on the earth Z axis, so with yaw forced
  # to avgYaw, calculate roll such that corrected roll + avgYaw = actual roll
  if abs(pitch) > pthresh
    # project body Y axis to earth xy plane to get roll angle
    quat = quaternion(q(1),q(2),q(3),q(4));
    [axis, theta] = q2rot(inv(unit(quat)));
    rmat = rotv(axis', theta);
    y_x = rmat(1,2);
    y_y = rmat(2,2);
    act_roll = -atan2(y_y, y_x);
    
    # direction of z axis flips on downline?
    if pitch < 0
      act_roll = -act_roll;
    endif
     
    # corrected roll = actual roll - avgYaw
    roll = rad2deg(act_roll); # - avgYaw;
    
    # wrap roll to [-180,180] degrees
    roll = wrap(roll);
    yaw = avgYaw;
##    printf("roll: %5.3f, pitch: %5.3f, yaw: %5.3f\n", roll, pitch, yaw);
  endif
endfunction
