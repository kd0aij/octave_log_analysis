function quat = euler2quat(roll, pitch, yaw)
  rq = rot2q([1,0,0], deg2rad(roll));
  pq = rot2q([0,1,0], deg2rad(pitch));
  yq = rot2q([0,0,1], deg2rad(yaw));
  quat = yq * pq * rq;
endfunction
