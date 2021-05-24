function angle = wrap(angle)
  # wrap angle to (-180,180]
  while angle > 180
    angle -= 360;
  endwhile
  while angle <= -180
    angle += 360;
  endwhile
endfunction
