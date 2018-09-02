function plot_tseg_color2(startTime, endTime, data, ...
  fignum=1, label='', boxCenter=[39.843,-105.2125], levelThresh=15)

# data contains fields: 
#         1    2    3    4     5      6    7
# timestamp, Lat, Lng, Alt, Roll, Pitch, Yaw,  
#    8     9    10    11    12    13  14  15  16
# GyrX, GyrY, GyrZ, AccX, AccY, AccZ, VE, VN, VD

# it appears that GPS location is NOT accurate when POS data starts
# skip the first 30 seconds of data
ts = data(:,1);

# find the record numbers for start and end times
startIndex = 1;
while (ts(startIndex) < startTime)
  startIndex += 1;
endwhile

endIndex = startIndex;
while (ts(endIndex) < endTime)
  endIndex += 1;
endwhile
printf("POS start/end indices: %d, %d\n", startIndex, endIndex);

# extract gyro rate vectors
tsp = ts(startIndex:endIndex);
gx = data(startIndex:endIndex,8);
gy = data(startIndex:endIndex,9);
gz = data(startIndex:endIndex,10);

lat=data(startIndex:endIndex,2);
lon=data(startIndex:endIndex,3);
z = data(startIndex:endIndex,4);

# convert to meters from start position
x=lon-boxCenter(2);
y=lat-boxCenter(1);
x=x*53*5280/3.28;
y=y*69*5280/3.28;
z -= 1808;

# rotate parallel to runway
a=16*(pi/180);
xyzr=[cos(a)*x.-sin(a)*y, sin(a)*x.+cos(a)*y, z];

# assign colors representing roll angle
# roll range is (-180,180] degrees
roll  = data(startIndex:endIndex,5);
pitch = data(startIndex:endIndex,6);
yaw   = data(startIndex:endIndex,7);
# convert yaw from [0,360] to [-180,180] (with North at 0)
for i = 1:length(yaw)
  if yaw > 180
    yaw = yaw - 360;
  endif
endfor

colors = ones(length(roll),3);
red = hsv2rgb([0,1,1]);
yellow = hsv2rgb([.13,1,1]);
green = hsv2rgb([.25,1,1]);
greeni = hsv2rgb([.3,1,1]);
blue = hsv2rgb([.60,1,1]);
magenta = [1,0,1];

for i = 1:length(colors)
  if abs(roll(i)) < levelThresh
    # level
    colors(i,:) = green;
  elseif abs(roll(i)-180) < levelThresh
    # inverted
    colors(i,:) = greeni;
  elseif abs(roll(i)-90) < levelThresh
    # right knife edge
    colors(i,:) = yellow;
  elseif abs(roll(i)+90) < levelThresh
    # left knife edge
    colors(i,:) = yellow;
  else
    colors(i,:) = red;
  endif
endfor
#colors = hsv2rgb(colors);
sizes = 10 * ones(length(colors),1);

# plot 5 second time intervals
thacks = [];
for i = 1:125:length(tsp)
  thacks = [thacks tsp(i)];
endfor

fignum
figure(fignum, 'position', [100,100,800,800])
subplot(2,2,1)
c1=1;
c2=2;
scatter(xyzr(:,c1),xyzr(:,c2),sizes,colors,'filled')
axis equal
limits = axis();
timeHacks(limits, xyzr, c1, c2, blue);
grid on
title (sprintf("Plan view"))
xlabel "east (m)"
ylabel "north (m)"

subplot(2,2,2)
c1=1;
c2=3;
scatter(xyzr(:,1),xyzr(:,3),sizes,colors,'filled');
axis equal
limits = axis();
timeHacks(limits, xyzr, c1, c2, blue);
grid on
title (sprintf("North elevation"))
xlabel "east (m)"
ylabel "alt (m)"

subplot(2,2,3)
c1=2;
c2=3;
scatter(xyzr(:,2),xyzr(:,3),sizes,colors,'filled');
axis equal
limits = axis();
timeHacks(limits, xyzr, c1, c2, blue);
grid on
title (sprintf("East elevation"))
xlabel "north (m)"
ylabel "alt (m)"

subplot(2,2,4)
plot(tsp, roll, 'or', tsp, pitch, 'ok', tsp, yaw-180, 'om');
limits=axis();
xoffset = (limits(2)-limits(1))/(length(thacks)*4)
yoffset = limits(3) + (limits(4)-limits(3))/40
hold on
for i = 1:length(thacks)
  plot([thacks(i) thacks(i)],limits(3:4),'-b');
  text(thacks(i)-xoffset,yoffset,num2str(i-1))
endfor
yGrid = [levelThresh 45 90 180-levelThresh 180];
for i = 1:length(yGrid)
  plot([limits(1) limits(2)],[-yGrid(i),-yGrid(i)],'-b');
  plot([limits(1) limits(2)],[ yGrid(i), yGrid(i)],'-b');
endfor
xticks(thacks);
xlabels={};
for i=1:length(thacks)
  xlabels(i) = sprintf("%4.1f",thacks(i));
endfor
xticklabels(xlabels);
yticks([-yGrid yGrid]);
axis([tsp(1),tsp(end),limits(3),limits(4)])
grid off

title (sprintf("roll, pitch, yaw"))
xlabel "time"
ylabel "degrees)"
legend("roll","pitch","yaw")

print (sprintf("%s_maneuver_%d.jpg", label, fignum), "-S1080,1080")

figure(fignum+1, 'position', [200,200,800,800])
subplot(2,1,1)
plot(tsp, unwrap(roll,180), '.r', tsp, unwrap(yaw-180,180), '.m', tsp, pitch, '.k');
limits=axis();
xoffset = (limits(2)-limits(1))/(length(thacks)*4)
yoffset = limits(3) + (limits(4)-limits(3))/40
hold on
for i = 1:length(thacks)
  plot([thacks(i) thacks(i)],limits(3:4),'-b');
  text(thacks(i)-xoffset,yoffset,num2str(i-1))
endfor
xticks(thacks);
xlabels={};
for i=1:length(thacks)
  xlabels(i) = sprintf("%4.1f",thacks(i));
endfor
xticklabels(xlabels);
yoff = 180*round(limits(3) / 180);
nwraps = round((limits(4)-limits(3)) / 180);
yTicks = [];
yGrid = [levelThresh 45 90 180-levelThresh 180];
for w = 1:nwraps
  for i = 1:length(yGrid)
    plot([limits(1) limits(2)],[ yoff+yGrid(i), yoff+yGrid(i)],'-b');
  endfor
  yTicks = [yTicks yoff+[0 45 90]];
  yoff += 180;
endfor
yticks(yTicks);
axis([tsp(1),tsp(end),limits(3),limits(4)])
grid off

title (sprintf("(unwrapped roll), pitch, yaw"))
xlabel "time"
ylabel "degrees)"
legend("roll","yaw","pitch")

subplot(2,1,2)
plot(tsp,rad2deg(gx),'or',tsp,rad2deg(gz),'om',tsp,rad2deg(gy),'ok')
limits=axis();
xoffset = (limits(2)-limits(1))/(length(thacks)*4)
yoffset = limits(3) + (limits(4)-limits(3))/40
hold on
for i = 1:length(thacks)
  plot([thacks(i) thacks(i)],limits(3:4),'-b');
  text(thacks(i)-xoffset,yoffset,num2str(i-1))
endfor
xticks(thacks);
xlabels={};
for i=1:length(thacks)
  xlabels(i) = sprintf("%4.1f",thacks(i));
endfor
xticklabels(xlabels);
axis([tsp(1),tsp(end),limits(3),limits(4)])
grid on

title ("roll, pitch, yaw rates")
xlabel "time"
ylabel "deg/sec)"
legend("roll","yaw","pitch")

print (sprintf("%s_RPY_%d.jpg", label, fignum), "-S1920,960")

endfunction

function timeHacks(limits, xyzr, c1, c2, color)
  hold on
  offset = (limits(4)-limits(3))/20;
  hackIndex = 0;
  for i = 1:125:length(xyzr)
    scatter(xyzr(i,c1),xyzr(i,c2),[10],color)
    text(xyzr(i,c1)+offset,xyzr(i,c2)+offset,num2str(hackIndex))
    hackIndex += 1;
  endfor
  hold off  
endfunction

##scatter3(xyzr(:,1),xyzr(:,2),xyzr(:,3),sizes,colors,'filled')
##hold on
##scatter3(xyzr(1,1),xyzr(1,2),xyzr(1,3),[20],[0,0,0],'filled')
##hold off
##
##axis equal
##grid on
##title (sprintf("3D view", label))
##xlabel "east (m)"
##ylabel "north (m)"
##zlabel "alt (m)"
##print (sprintf("%s_maneuver_%d.jpg", label, index1))
##scatter3(xyzr(:,1),xyzr(:,2),xyzr(:,3),sizes,colors,'filled')
##hold on
##scatter3(xyzr(1,1),xyzr(1,2),xyzr(1,3),[20],[0,0,0],'filled')
##hold off
##
##xlabel('x')
##ylabel("y")
##zlabel("z")
##axis equal
##grid on
##title (sprintf("3D view", label))
##xlabel "east (m)"
##ylabel "north (m)"
##zlabel "alt (m)"
