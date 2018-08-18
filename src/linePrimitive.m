function p = linePrimitive(o, v, dt)
  # generate 3D point on line specified by origin, unit vector and dt
  # p, o and v are row vectors with dimensions (1,3)
  p = dt * v + o;
endfunction
