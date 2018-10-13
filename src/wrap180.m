function result = wrap180(deg)
    # wrap to [-180,180]
    if deg > 0 
      sign = 1;
    else
      sign = -1;
    endif
    result = deg;
    while abs(result) > 180
      result -= sign * 360;
    endwhile
endfunction
