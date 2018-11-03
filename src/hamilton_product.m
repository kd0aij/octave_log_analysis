function vp = hamilton_product(quat, v)
  # rotate a 3-vector 
  qv = quaternion(0, v(1), v(2), v(3));
  qp = quat * qv * inv(quat);
  vp = [qp.i qp.j qp.k];
endfunction
