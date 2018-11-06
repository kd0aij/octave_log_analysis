function plot_tseg_color2(startTime, endTime, data, ...
  fignum=1, label='', origin=[39.8420194 -105.2123333 1808], 
  rollTolerance=15, posIndex=2, runwayNorth=16, pThresh=80, plotTitle='')
  
# data contains fields: 
#         1    2    3    4     5      6    7
# timestamp, Lat, Lng, Alt, Roll, Pitch, Yaw,  

#    8     9    10    11    12    13  14  15  16
# GyrX, GyrY, GyrZ, AccX, AccY, AccZ, VE, VN, VD

# 17:20, 21, 22, 23
# Q1-4,   R,  P,  Y (corrected Euler angles "fixed xyz")

#  24,  25,  26
# Lat, Lng, Alt (GPS based)

# posIndex defaults to 2 for POS based Lat,Lng,Alt
# if those are in error, set posIndex to 24 to fall back to GPS samples

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
##printf("POS start/end indices: %d, %d\n", startIndex, endIndex);

# extract gyro rate vectors
tsp = ts(startIndex:endIndex);
gx = data(startIndex:endIndex,8);
gy = data(startIndex:endIndex,9);
gz = data(startIndex:endIndex,10);
##[gx gy gz] = data(startIndex:endIndex,8:10);

lat=data(startIndex:endIndex,posIndex);
lon=data(startIndex:endIndex,posIndex+1);
z = data(startIndex:endIndex,posIndex+2);

# convert to meters from origin
# and rotate parallel to runway
xyzr = lla2xyz([lat lon z], -runwayNorth, origin);

# assign colors representing roll angle
# roll range is (-180,180] degrees
##roll  = data(startIndex:endIndex,5);
##pitch = data(startIndex:endIndex,6);
##yaw   = data(startIndex:endIndex,7);
roll  = data(startIndex:endIndex,24);
pitch = data(startIndex:endIndex,25);
yaw   = data(startIndex:endIndex,26);

# convert hdg from [0,360] (positive CW) to 
# [180,-180] (positive CCW) (with North at 0)
for i = 1:length(yaw)
  if yaw(i) > 180
    yaw(i) = 360 - yaw(i);
  else
    yaw(i) = -yaw(i);
  endif
endfor

colors = ones(length(roll),3);
red = hsv2rgb([0,1,1]);
yellow = hsv2rgb([.13,1,1]);
green = hsv2rgb([.25,1,1]);
greeni = hsv2rgb([.25,1,.8]);
blue = hsv2rgb([.60,1,1]);
magenta = [1,0,1];
rollErr = [];

for i = 1:length(colors)
  if abs(roll(i)) < rollTolerance
    # level
    colors(i,:) = green;
  elseif abs(roll(i)-180) < rollTolerance
    # inverted
    colors(i,:) = greeni;
  elseif abs(roll(i)+180) < rollTolerance
    # inverted
    colors(i,:) = greeni;
  elseif abs(roll(i)-90) < rollTolerance
    # right knife edge
    colors(i,:) = yellow;
  elseif abs(roll(i)+90) < rollTolerance
    # left knife edge
    colors(i,:) = yellow;
  else
    colors(i,:) = red;
    rollErr = [rollErr i];
  endif
endfor
sizes = 10 * ones(length(colors),1);

# plot 5 second time intervals
thacks = [];
for i = 1:125:length(tsp)
  thacks = [thacks tsp(i)];
endfor

fignum
figure(fignum, 'position', [100,100,800,800])
subplot(2,2,1)
scatterPlot(xyzr, 1, 2, sizes, colors, blue, [-350 350])
title (sprintf("Plan view\n%s", plotTitle))
xlabel "east (m)"
ylabel "north (m)"

subplot(2,2,2)
scatterPlot(xyzr, 1, 3, sizes, colors, blue, [-350 350])
title (sprintf("North elevation"))
xlabel "east (m)"
ylabel "alt (m)"

subplot(2,2,3)
##scatter(xyzr(:,2),xyzr(:,3),sizes,colors,'filled')
scatterPlot(xyzr, 2, 3, sizes, colors, blue, [0 300])
title (sprintf("East elevation"))
xlabel "north (m)"
ylabel "alt (m)"

subplot(2,2,4)
plot(tsp, roll, 'or', tsp, pitch, '.-k', tsp, yaw, 'om');
limits=axis();
xoffset = (limits(2)-limits(1))/(length(thacks)*4);
yoffset = limits(3) + (limits(4)-limits(3))/40;
hold on
for i = 1:length(thacks)
  plot([thacks(i) thacks(i)],limits(3:4),'-b');
  text(thacks(i)-xoffset,yoffset,num2str(i-1))
endfor
yGrid = [rollTolerance 45 90 180-rollTolerance 180];
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

figure(fignum+1, 'position', [900,100,800,800])
subplot(2,1,1)
plot(tsp, (180/pi)*unwrap(roll*pi/180), '.-r', tsp, pitch, '.-k', tsp, (180/pi)*unwrap(yaw*pi/180), '.-m');
# highlight abs(pitch) > pThresh
hold on
xxx = find(abs(pitch)>pThresh);
plot(tsp(xxx), pitch(xxx), 'ok');
# highlight roll error > rollTolerance
if length(rollErr) > 0
  plot(tsp(rollErr), rad2deg(unwrap(deg2rad(roll(rollErr)))), 'or');
endif
limits=axis();
xoffset = (limits(2)-limits(1))/(length(thacks)*4);
yoffset = limits(3) + (limits(4)-limits(3))/40;
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
yGrid = [rollTolerance 45 90 180-rollTolerance 180+rollTolerance 270-rollTolerance 270+rollTolerance ];
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

title (sprintf("unwrapped roll, pitch, unwrapped yaw"))
##title (sprintf("roll, pitch, yaw"))
xlabel "time"
ylabel "degrees)"
legend("roll","pitch","yaw")

subplot(2,1,2)
plot(tsp,rad2deg(gx),'.-r',tsp,rad2deg(gy),'.-k',tsp,rad2deg(gz),'.-m')
limits=axis();
xoffset = (limits(2)-limits(1))/(length(thacks)*4);
yoffset = limits(3) + (limits(4)-limits(3))/40;
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
legend("roll","pitch","yaw")

print (sprintf("%s_RPY_%d.jpg", label, fignum), "-S1080,540")
hgsave (sprintf("%s_RPY_%d.ofig", label, fignum))

figure(fignum)
print (sprintf("%s_maneuver_%d.jpg", label, fignum), "-S800,800")
hgsave (sprintf("%s_maneuver_%d.ofig", label, fignum))

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

function scatterPlot(xyzr, c1, c2, sizes, colors, color, xlim)
  scatter(xyzr(:,c1),xyzr(:,c2),sizes,colors,'filled')
  axis equal
  axis(xlim)
  limits = axis();
  timeHacks(limits, xyzr, c1, c2, color);
  grid on
endfunction
