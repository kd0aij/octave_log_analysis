function deg = wrap180(deg, hyst=0)
  if isscalar(deg)
    # wrap to [-180,180]
    while deg > 180+hyst
      deg -= 360;
    endwhile
    while deg < -180-hyst
      deg += 360;
    endwhile
  else
    while any(deg > (180 + hyst))
      idx = deg > (180 + hyst);
      deg(idx) -= 360;
    endwhile
    while any(deg < (-180 - hyst))
      idx = deg < (-180 - hyst);
      deg(idx) += 360;
    endwhile
  endif
endfunction
