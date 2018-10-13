function [r, p, y] = attFromQuat(q, avgYaw, pthresh)
  # input q is an array [w x y z] 
  # q = w + xi + yj + zk
  # result is xyz-fixed Euler angles in degrees
  # this rotation is from body to earth-fixed (NED)
  [r p y] = quat2euler(q);
  
  # avoid the singularity at abs(pitch)==90
  # for pattern judging, we need roll w.r.t. the maneuver heading, which
  # is input as avgYaw. But since it's the trajectory which must be vertical
  # on up/down lines, pitch and yaw will be dependent on wind, not necessarily
  # +/-90 and avgYaw, respectively...
  # Assume that pitch angles > pthresh degrees are verticals, and force
  # yaw to avgYaw
  # When vertical, both roll and yaw act on the earth Z axis, so with yaw forced
  # to avgYaw, calculate roll such that corrected roll + avgYaw = actual roll
  if abs(p) > pthresh
    quat = quaternion(q(1),q(2),q(3),q(4));
    
    printf("roll: %5.3f, pitch: %5.3f, yaw: %5.3f, avgYaw: %5.3f\n", r, p, y, avgYaw);
      
    # remove avgYaw
    aq = inv(rot2q([0,0,1],deg2rad(avgYaw))) * quat;
    # remove pitch
    aq = inv(rot2q([0,1,0],deg2rad(real(p)))) * aq;
    # remaining rotation is roll
    [axisr, thetar] = q2rot(unit(aq));
    if p < 0
      thetar *= -1;
    endif
    r = wrap(rad2deg(thetar));
    y = avgYaw;
    printf("corrected roll: %5.3f, pitch: %5.3f, yaw: %5.3f\n", r, p, y);
      
  endif
endfunction
