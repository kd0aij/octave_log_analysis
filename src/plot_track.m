function plot_track(index1, index2, segments, POS)

if (index1 > 1)
  startIndex = segments{index1-1}(5);
else
  startIndex = segments{index1}(4);
endif

# truncate first 10 seconds of POS data (250 samples)
if (startIndex < 250) 
  startIndex = 250;
endif

if (index2 < length(segments))
  endIndex = segments{index2+1}(4);
else
  endIndex = segments{index2}(5);
endif

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

figure 1
plot(xyzr(:,1),xyzr(:,2))
axis equal
grid on
title "east, north in meters"

figure 2
plot(xyzr(:,1),xyzr(:,3));
axis equal
grid on
title "east, alt in meters"

figure 3
plot3(xyzr(:,1),xyzr(:,2),xyzr(:,3))
xlabel('x')
ylabel("y")
zlabel("z")
axis equal
grid on
title "rotated 3D"
