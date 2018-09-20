function [roll, pitch, yaw] = attFromQ(q)
  [roll, pitch, yaw] = attFromQuat([q.w q.x q.y q.z]);
endfunction
