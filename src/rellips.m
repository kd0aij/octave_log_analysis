function [rx, ry, rz] = rellips(R, ex, ey, ez, T)
[N,M]=size(ex);
rx = zeros(N,M);
ry = zeros(N,M);
rz = zeros(N,M);
for row=1:N
  for col=1:M
    rv = R * [ ex(row,col); ey(row,col); ez(row,col) ];
    rx(row,col) = rv(1) + T(1);
    ry(row,col) = rv(2) + T(2);
    rz(row,col) = rv(3) + T(3);
  end
end
endfunction
