function plot_track2(index1, index2, POS, fnum=0)

startIndex = index1
endIndex = index2

lat=POS.data(startIndex:endIndex,3);
lon=POS.data(startIndex:endIndex,4);
z = POS.data(startIndex:endIndex,5);

# convert to meters from start position
x=lon-lon(1);
y=lat-lat(1);
x=x*53*5280/3.28;
y=y*69*5280/3.28;
z -= z(1);

# rotate parallel to runway
a=16*(pi/180);
xyzr=[cos(a)*x.-sin(a)*y, sin(a)*x.+cos(a)*y, z];

figure (fnum)
clf
plot3(xyzr(:,1),xyzr(:,2),xyzr(:,3))
hold
plot(xyzr(1,1),xyzr(1,2),"*k")
xlabel('x')
ylabel("y")
zlabel("z")
axis equal
grid on
title (sprintf("3D view"))
xlabel "east (m)"
ylabel "north (m)"
zlabel "alt (m)"
