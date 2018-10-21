function deg = m2dLng(meters, lat)
  deg = meters / (111111.0 * cos(deg2rad(lat)));
endfunction
