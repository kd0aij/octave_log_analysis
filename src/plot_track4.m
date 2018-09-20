function plot_track4(slBegin, index, xyz, fnum=0)

figure (1)
clf
plot3(xyz(slBegin:index,1),xyz(slBegin:index,2),xyz(slBegin:index,3))
hold
plot3(xyz(1,1),xyz(1,2),xyz(1,3),"*k")
xlabel('x')
ylabel("y")
zlabel("z")
axis equal
grid on
title (sprintf("3D view"))
xlabel "east (m)"
ylabel "north (m)"
zlabel "alt (m)"

input("paused");
