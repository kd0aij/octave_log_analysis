function R = q2rotm(q)
  # convert quaternion to rotation matrix
  # input q is an array [w x y z] or quaternion object: q.w q.i q.j q.k
  if isa(q,'quaternion') 
    w = q.w;
    x = q.x;
    y = q.y;
    z = q.z;
  else
    w = q(1);
    x = q(2);
    y = q(3);
    z = q(4);
  endif
  w2 = w*w;
  x2 = x*x;
  y2 = y*y;
  z2 = z*z;
  R =[ w2+x2-y2-z2 2*x*y-2*w*z 2*x*z+2*w*y;
       2*x*y+2*w*z w2-x2+y2-z2 2*y*z-2*w*x;
       2*x*z-2*w*y 2*y*z+2*w*x w2-x2-y2+z2
     ];
end
