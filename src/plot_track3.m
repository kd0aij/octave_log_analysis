function plot_track(index1, segments, POS, label='', boxCenter=[39.843,-105.2125])
# boxCenter is the (lat,lon) of the box center  

# it appears that GPS location is accurate when POS data starts
pts = POS.data(:,1);
iniTime = pts(1);

# vehicle has just departed "straight and level"
startTime = segments{index1}(1);

if (startTime < iniTime) 
  startTime = iniTime;
endif

# if this isn't the last maneuver, include the straight and level exit
if (index1 < length(segments))
  endTime = segments{index1+1}(2);
else
  endTime = segments{index1}(2);
endif

# find the record numbers for start and end times
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
x=lon-boxCenter(2);
y=lat-boxCenter(1);
x=x*53*5280/3.28;
y=y*69*5280/3.28;
z -= 1808;

# rotate parallel to runway
a=16*(pi/180);
xyzr=[cos(a)*x.-sin(a)*y, sin(a)*x.+cos(a)*y, z];

fignum = index1;
figure(fignum, 'position', [100,100,800,800])
subplot(2,2,1)
plot(xyzr(:,1),xyzr(:,2))
hold
plot(xyzr(1,1),xyzr(1,2),"*k")

axis equal
grid on
title (sprintf("Plan view"))
xlabel "east (m)"
ylabel "north (m)"

subplot(2,2,2)
plot(xyzr(:,1),xyzr(:,3));
hold
plot(xyzr(1,1),xyzr(1,3),"*k")
axis equal
grid on
title (sprintf("North elevation", label))
xlabel "east (m)"
ylabel "alt (m)"

subplot(2,2,3)
plot(xyzr(:,2),xyzr(:,3));
hold
plot(xyzr(1,2),xyzr(1,3),"*k")
axis equal
grid on
title (sprintf("East elevation", label))
xlabel "north (m)"
ylabel "alt (m)"

subplot(2,2,4)
plot3(xyzr(:,1),xyzr(:,2),xyzr(:,3))
hold
plot(xyzr(1,1),xyzr(1,2),"*k")
xlabel('x')
ylabel("y")
zlabel("z")
axis equal
grid on
title (sprintf("3D view", label))
xlabel "east (m)"
ylabel "north (m)"
zlabel "alt (m)"
print (sprintf("%s_maneuver_%d.jpg", label, index1))

close
