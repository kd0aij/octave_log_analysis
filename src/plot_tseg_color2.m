function plot_tseg_color2(startTime, endTime, data,
  fignum=1, label='untitled', origin=[39.8420194 -105.2123333 1808], 
  rTol=15, posIndex=2, pilotNorth=16, pThresh=88, plotTitle='')
  
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
tsp = ts(startIndex:endIndex);
Nsamp = length(tsp)

# extract gyro rate vectors
gv = data(startIndex:endIndex,8:10);

lat=data(startIndex:endIndex,posIndex);
lon=data(startIndex:endIndex,posIndex+1);
z = data(startIndex:endIndex,posIndex+2);

# extract vNED and convert to vENU
vNED = data(startIndex:endIndex,14:16);
vENU = [vNED(:,2) vNED(:,1) -vNED(:,3)];

# extract quaternion attitude
quat = data(startIndex:endIndex,17:20);

# convert to meters from origin
xyz = lla2xyz([lat lon z], 0, origin);
# and rotate parallel to runway
xyzr = lla2xyz([lat lon z], pilotNorth, origin);

# assign colors representing roll angle
# roll range is (-180,180] degrees
roll  = zeros(Nsamp, 1);
pitch = zeros(Nsamp, 1);
hdg   = data(startIndex:endIndex,7);
# convert hdg from [0,360] (positive CW) to 
# [180,-180] (positive CW) (with North at 0)
yaw = hdg2yaw(hdg);

e_roll = data(startIndex:endIndex,5);
e_pitch = data(startIndex:endIndex,6);

# compute roll/pitch in maneuver plane
mhdg = zeros(Nsamp, 1);
mplanes = [];
chdg = pilotNorth + 90;
disp(sprintf("Runway heading (CW from North) is %5.1f (East) / %5.1f (West)", 
             chdg, 180+chdg));
onVertical = 0;
avghdg = chdg;
ghdg = chdg;
then = time;
for idx = 1:Nsamp
  [roll(idx) pitch(idx) wca] = maneuver_roll_pitch(avghdg, quat(idx,:), pThresh);
  mhdg(idx) = avghdg;
  if onVertical
    # hysteresis
    if abs(pitch(idx)) < (pThresh - 0.5)
      onVertical = 0;
      # on exit from vertical line
      # record maneuver plane
      ghdg = atan2d(vENU(idx,1),vENU(idx,2));
      if abs(wrap180(ghdg-chdg)) > 90
        avghdg = wrap180(180+chdg);
      else
        avghdg = chdg;
      endif
      mplane.hdg = avghdg;
      mplane.pos = xyz(idx,:);
      mplanes = [mplanes mplane];
      
      disp(sprintf("t: %5.1f pitch: %3.1f, course: %4.1f, gspd: %3.1f, vspd: %3.1f, wca: %3.1f",
              tsp(idx), pitch(idx), avghdg, 
              vectorNorm(vENU(idx,1:2)), vectorNorm(vENU(idx,3)),
              wca));
    endif
  else
    # while not on vertical line; update heading
##    avghdg += .05 * (chdg - avghdg);
    # specify maneuver heading as current ground course
    ghdg = atan2d(vENU(idx,1),vENU(idx,2));
    if abs(wrap180(ghdg-chdg)) > 90
      avghdg = wrap180(180+chdg);
    else
      avghdg = chdg;
    endif
    
    # vertical line if pitch > threshold
    if abs(pitch(idx)) > pThresh
      onVertical = 1;
      # on entry to vertical line:
      # record maneuver plane
      mplane.hdg = avghdg;
      mplane.pos = xyz(idx,:);
      mplanes = [mplanes mplane];
      
      disp(sprintf("t: %5.1f pitch: %3.1f, course: %4.1f, gspd: %3.1f, vspd: %3.1f, wca: %3.1f",
              tsp(idx), pitch(idx), avghdg, 
              vectorNorm(vENU(idx,1:2)), vectorNorm(vENU(idx,3)),
              wca));
    endif
  endif


##  if (not(onVertical) && (abs(pitch(idx)) > pThresh))
##    onVertical = 1;
##    # record maneuver plane
##    mplane.hdg = rhdg;
##    mplane.pos = xyz(idx,:);
##    mplanes = [mplanes mplane];
##    
##    disp(sprintf("t: %5.1f pitch: %3.1f, course: %4.1f, gspd: %3.1f, vspd: %3.1f, wca: %3.1f",
##            tsp(idx), pitch(idx), rhdg, 
##            vectorNorm(vENU(idx,1:2)), vectorNorm(vENU(idx,3)),
##            wca));
##  elseif abs(pitch(idx)) < pThresh
##    onVertical = 0;
##    # specify maneuver heading as current ground course
##    # this is NED, so y is the first value
##    rhdg = atan2d(vENU(idx,1),vENU(idx,2));
##  endif
endfor
disp(sprintf("maneuver_roll_pitch elapsed time: %f", time-then));

figure(50)
plot3Dline(xyz, '-');
axis equal
grid on
rotate3d on
title('unrotated xyz')
xlabel('East')
ylabel('True North')
zlabel('Alt')

# plot maneuver plane: 50x50 meters, vertical, rotated to maneuver heading
hold on
limits=axis();
for idx = 1:length(mplanes)
  x = [0; 0];
  y = [-50; 50];
  # rotate to heading
  theta = 180 - mplanes(idx).hdg;
  xyr = [cosd(theta)*x.-sind(theta)*y, sind(theta)*x.+cosd(theta)*y];
  # translate to vehicle position
  xyr(:,1:2) += mplanes(idx).pos(1:2);
  [xx yy] = meshgrid(xyr(:,1),xyr(:,2));
  zz = [-100 -100; 100 100] + mplanes(idx).pos(3);
  s1 = surf(xx,yy',zz); #, 'Edgecolor', 'none');
  set(s1,'facealpha',0.2);
##  shading interp;
endfor

figure(51)
plot(tsp, (e_roll), 'o', tsp, (roll), tsp, e_pitch, 'o', tsp, pitch, 
     tsp, yaw, tsp, mhdg)
title('Euler RPY vs. maneuver RPY')
legend(['eroll'; 'roll'; 'epitch'; 'pitch'; 'eyaw'; 'mhdg'])
axis tight
grid minor

colors = ones(length(roll),3);
red = hsv2rgb([0,1,1]);
yellow = hsv2rgb([.13,1,1]);
green = hsv2rgb([.25,1,1]);
greeni = hsv2rgb([.25,1,.8]);
blue = hsv2rgb([.60,1,1]);
magenta = [1,0,1];
rollErr = [];

for idx = 1:length(colors)
  if rollCheck(roll(idx), e_roll(idx), 0, rTol) #abs(roll(idx)) < rTol
    # level
    colors(idx,:) = green;
  elseif rollCheck(roll(idx), e_roll(idx), -180, rTol) #abs(roll(idx)-180) < rTol
    # inverted
    colors(idx,:) = greeni;
  elseif rollCheck(roll(idx), e_roll(idx), 180, rTol) #abs(roll(idx)+180) < rTol
    # inverted
    colors(idx,:) = greeni;
  elseif rollCheck(roll(idx), e_roll(idx), -90, rTol) #abs(roll(idx)-90) < rTol
    # right knife edge
    colors(idx,:) = yellow;
  elseif rollCheck(roll(idx), e_roll(idx), 90, rTol) #abs(roll(idx)+90) < rTol
    # left knife edge
    colors(idx,:) = yellow;
  else
    colors(idx,:) = red;
    rollErr = [rollErr idx];
  endif
endfor
sizes = 10 * ones(length(colors),1);

# plot 5 second time intervals
thacks = [];
for idx = 1:125:Nsamp
  thacks = [thacks tsp(idx)];
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
for idx = 1:length(thacks)
  plot([thacks(idx) thacks(idx)],limits(3:4),'-b');
  text(thacks(idx)-xoffset,yoffset,num2str(idx-1))
endfor
yGrid = [rTol 45 90 180-rTol 180];
for idx = 1:length(yGrid)
  plot([limits(1) limits(2)],[-yGrid(idx),-yGrid(idx)],'-b');
  plot([limits(1) limits(2)],[ yGrid(idx), yGrid(idx)],'-b');
endfor
xticks(thacks);
xlabels={};
for idx=1:length(thacks)
  xlabels(idx) = sprintf("%4.1f",thacks(idx));
endfor
xticklabels(xlabels);
yticks([-yGrid yGrid]);
axis([tsp(1),tsp(end),limits(3),limits(4)])
grid off

title (sprintf("roll, pitch, yaw"))
xlabel "time"
ylabel "degrees)"
legend("roll","pitch","yaw")

uroll = unwrapd(roll);
uyaw  = unwrapd(yaw);

figure(fignum+1, 'position', [900,100,800,800])
subplot(2,1,1)
plot(tsp, uroll, '.-r', tsp, pitch, '.-k', tsp, uyaw, '.-m');
# highlight abs(pitch) > pThresh
hold on
xxx = find(abs(pitch)>pThresh);
plot(tsp(xxx), pitch(xxx), 'ok');
# highlight roll error > rTol
if length(rollErr) > 0
  plot(tsp(rollErr), uroll(rollErr), 'or');
endif
limits=axis();
xoffset = (limits(2)-limits(1))/(length(thacks)*4);
yoffset = limits(3) + (limits(4)-limits(3))/40;
for idx = 1:length(thacks)
  plot([thacks(idx) thacks(idx)],limits(3:4),'-b');
  text(thacks(idx)-xoffset,yoffset,num2str(idx-1))
endfor
xticks(thacks);
xlabels={};
for idx=1:length(thacks)
  xlabels(idx) = sprintf("%4.1f",thacks(idx));
endfor
xticklabels(xlabels);
yoff = 360*round(limits(3) / 360);
nwraps = round((limits(4)-limits(3)) / 360) - round(yoff/360);
yTicks = [];
yGrid = [-rTol rTol 45 90-rTol 90+rTol 135 
         180-rTol 180+rTol 225 270-rTol 270+rTol 315];
yGridLabels = {'up', 'up', 'r45', 'rKE', 'rKE', 'ir45', 
               'inv', 'inv', 'il45', 'lKE', 'lKE', 'l45'};
yTickLabels = [];
for w = 1:nwraps
##  for idx = 1:length(yGrid)
##    plot([limits(1) limits(2)],[ yoff+yGrid(idx), yoff+yGrid(idx)],'-b');
##  endfor
  yTicks = [yTicks yoff+yGrid];
  yTickLabels = [yTickLabels yGridLabels];
  yoff += 360;
endfor
yticks(yTicks);
yticklabels(yTickLabels);
axis([tsp(1),tsp(end),limits(3),limits(4)])
grid on

title (sprintf("unwrapped roll, pitch, unwrapped yaw"))
##title (sprintf("roll, pitch, yaw"))
xlabel "time"
ylabel "degrees)"
legend("roll","pitch","yaw")

subplot(2,1,2)
plot(tsp,rad2deg(gv),'.-')
limits=axis();
xoffset = (limits(2)-limits(1))/(length(thacks)*4);
yoffset = limits(3) + (limits(4)-limits(3))/40;
hold on
for idx = 1:length(thacks)
  plot([thacks(idx) thacks(idx)],limits(3:4),'-b');
  text(thacks(idx)-xoffset,yoffset,num2str(idx-1))
endfor
xticks(thacks);
xlabels={};
for idx=1:length(thacks)
  xlabels(idx) = sprintf("%4.1f",thacks(idx));
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
  for idx = 1:125:length(xyzr)
    scatter(xyzr(idx,c1),xyzr(idx,c2),[10],color)
    text(xyzr(idx,c1)+offset,xyzr(idx,c2)+offset,num2str(hackIndex))
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

function good = rollCheck(mroll, eroll, center, tol)
  good = (abs(mroll-center) < tol) || (abs(eroll-center) < tol);
##  good = (abs(mroll-center) < tol);
endfunction

function u = unwrapd(angled)
  u = rad2deg(unwrap(deg2rad(angled)));  
endfunction
