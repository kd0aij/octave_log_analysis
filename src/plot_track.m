function plot_track(index1, index2, segments, POS, label='', plot3d=0)

pts = POS.data(:,1);
iniTime = pts(1);

startTime = segments{index1}(1);

if (startTime < iniTime) 
  startTime = iniTime;
endif

if (index2 < length(segments))
  endTime = segments{index2+1}(2);
else
  endTime = segments{index2}(2);
endif

startIndex = 1;
while (pts(startIndex) < startTime)
  startIndex += 1;
endwhile

endIndex = startIndex;
while (pts(endIndex) < endTime)
  endIndex += 1;
endwhile

startIndex
endIndex

lat=POS.data(startIndex:endIndex,3);
lon=POS.data(startIndex:endIndex,4);
z = POS.data(startIndex:endIndex,5);

# convert to meters from start position
x=lon-POS.data(1,4);
y=lat-POS.data(1,3);
x=x*53*5280/3.28;
y=y*69*5280/3.28;
z -= 1808;

# rotate parallel to runway
a=16*(pi/180);
xyzr=[cos(a)*x.-sin(a)*y, sin(a)*x.+cos(a)*y, z];

fignum = 10 * index1;
figure(fignum)
plot(xyzr(:,1),xyzr(:,2))
hold
plot(xyzr(1,1),xyzr(1,2),"*k")

axis equal
grid on
title (sprintf("%s Top view, segments %d:%d", label, index1, index2))
xlabel "east (m)"
ylabel "north (m)"
print (sprintf("%s_top-%d_%d.jpg", label, index1, index2))

figure(fignum+1)
plot(xyzr(:,1),xyzr(:,3));
hold
plot(xyzr(1,1),xyzr(1,3),"*k")
axis equal
grid on
title (sprintf("%s Side view, segments %d:%d", label, index1, index2))
xlabel "east (m)"
ylabel "alt (m)"
print (sprintf("%s_side-%d_%d.jpg", label, index1, index2))

figure(fignum+2)
plot(xyzr(:,2),xyzr(:,3));
hold
plot(xyzr(1,2),xyzr(1,3),"*k")
axis equal
grid on
title (sprintf("%s End view, segments %d:%d", label, index1, index2))
xlabel "north (m)"
ylabel "alt (m)"
print (sprintf("%s_end-%d_%d.jpg", label, index1, index2))

if (plot3d)
figure 4
plot3(xyzr(:,1),xyzr(:,2),xyzr(:,3))
hold
plot(xyzr(1,1),xyzr(1,2),"*k")
xlabel('x')
ylabel("y")
zlabel("z")
axis equal
grid on
title (sprintf("%s 3D view, segments %d:%d", label, index1, index2))
xlabel "east (m)"
ylabel "north (m)"
zlabel "alt (m)"
print (sprintf("%s_3d-%d_%d.jpg", label, index1, index2))
endif