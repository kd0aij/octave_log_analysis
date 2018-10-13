function result = wrappedPitch(pitch)
    result = wrap180(pitch);
    # constrain to [-90,90]
    if result > 90
      result = 180 - result;
    elseif result < -90
      result = -180 - result;
    endif
endfunction
