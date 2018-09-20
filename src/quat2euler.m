function [roll pitch yaw] = quat2euler(q)
  # input q is an array [w x y z] 
  # q = w + xi + yj + zk
  # result is xyz-fixed Euler angles in degrees
  roll  = rad2deg(atan2(2*(q(1)*q(2) + q(3)*q(4)), 1 - 2*(q(2)*q(2) + q(3)*q(3))));
  pitch = rad2deg( asin(2*(q(1)*q(3) - q(4)*q(2))));
  yaw   = rad2deg(atan2(2*(q(1)*q(4) + q(2)*q(3)), 1 - 2*(q(3)*q(3) + q(4)*q(4))));
endfunction

