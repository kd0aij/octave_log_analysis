function plot_tseg_color2(startTime, endTime, data,
  mnum=0, label='untitled', origin=[39.8420194 -105.2123333 1808], 
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
             chdg, wrap180(180+chdg)));
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

fignum = mnum * 10;
figure(fignum++)
plot3Dline(xyz, '-');
axis equal
grid on
rotate3d on
title('unrotated xyz')
xlabel('East')
ylabel('True North')
zlabel('Alt')
# save figure
savefig("3D", label, mnum, 800, 800);

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

figure(fignum++)
plot(tsp, (e_roll), 'o', tsp, (roll), tsp, e_pitch, 'o', tsp, pitch, 
     tsp, yaw, tsp, mhdg)
title('Euler RPY vs. maneuver RPY')
legend(['eroll'; 'roll'; 'epitch'; 'pitch'; 'eyaw'; 'mhdg'])
axis tight
grid minor
# save figure
savefig("eulerVmp", label, mnum, 1080, 540);
fname = sprintf("%s_eulerVmp_%d", label, mnum);
disp(sprintf("saving 3D track display: %s", fname));
print ([fname ".jpg"], "-S1080,540")
hgsave ([fname ".jpg"])

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

if length(plotTitle) == 0
  plotTitle = sprintf("roll tolerance: %2.0f", rTol);
endif
fignum
figure(fignum++, 'position', [100,100,800,800])
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
scatterPlot(xyzr, 2, 3, sizes, colors, blue, [0 300])
title (sprintf("East elevation"))
xlabel "north (m)"
ylabel "alt (m)"

subplot(2,2,4)
plot(tsp, roll, 'or', tsp, pitch, '.-k', tsp, yaw, 'om');
limits=axis();
xoffset = (limits(2)-limits(1))/(length(thacks)*8);
yoffset = limits(3) + (limits(4)-limits(3))/80;
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

# save figure
savefig("maneuver", label, mnum, 800, 800);

uroll = unwrapd(roll);
wuroll = wrap180(uroll, rTol);

figure(fignum++, 'position', [900,100,800,800])
subplot(2,1,1)
ax = plotyy(tsp, wuroll, tsp, [pitch mhdg]);
rline = findobj(ax(1),'linestyle','-')
set(rline,"linestyle",'-')
set(rline,"marker",'.');
set(rline,"markersize",10)
hl = legend(ax(1), "roll", "pitch", "hdg")
legend(hl, "location", "northeastoutside");
axis(ax(1), [tsp(1) tsp(end) -180-rTol 180+rTol]);
axis(ax(2), [tsp(1) tsp(end) -180-rTol 180+rTol]);
grid(ax(1), 'on')
grid(ax(2), 'on')
set(ax(1),'ygrid','on')
set(ax(2),'ygrid','on')
set(ax(2),'xgrid','off')
hold on

### highlight abs(pitch) > pThresh
##xxx = find(abs(pitch)>pThresh);
##plot(ax(2), tsp(xxx), pitch(xxx), '.r');

# highlight roll error > rTol
if length(rollErr) > 0
  plot(ax(1), tsp(rollErr), roll(rollErr), 'ob');
endif

# plot x axis time hacks
xoffset = (limits(2)-limits(1))/(length(thacks)*4);
yoffset = limits(3) + (limits(4)-limits(3))/40;
xticks(ax(1), thacks);
xticks(ax(2), []);
xlabels={};
for idx=2:length(thacks)
  xlabels(idx) = sprintf("T%i:%4.1f", idx-1, thacks(idx));
endfor
xticklabels(xlabels);
yoff = 0;
nwraps = 1;
yTicks = [];
yGrid = [-180-rTol -180+rTol -135 -90-rTol -90+rTol -45 -rTol ...
         rTol 45 90-rTol 90+rTol 135 180-rTol 180+rTol];
yGridLabels = {'inv', 'inv', 'il45', 'lKE', 'lKE', 'l45', ...
               'up', 'up', 'r45', 'rKE', 'rKE', 'ir45', 'inv', 'inv'};
yTickLabels = [];
for w = 1:nwraps
  yTicks = [yTicks yoff+yGrid];
  yTickLabels = [yTickLabels yGridLabels];
  yoff += 360;
endfor
yticks(ax(1), yTicks);
yticklabels(ax(1), yTickLabels);

yTicks = [-180 -135 -90 -45 0 45 90 135 180];
yticks(ax(2), yTicks);


title (sprintf("roll, pitch, maneuver hdg"))
xlabel "time"
ylabel(ax(1), "roll attitude")
ylabel(ax(2), "pitch, hdg degrees")

subplot(2,1,2)
plot(tsp,rad2deg(gv),'.-')
limits=axis();
xoffset = (limits(2)-limits(1))/(length(thacks)*4);
yoffset = limits(3) + (limits(4)-limits(3))/40;
hold on
xticks(thacks);
xticklabels(xlabels);
axis([tsp(1),tsp(end),limits(3),limits(4)])
grid on

title ("roll, pitch, yaw rates")
xlabel "time"
ylabel "deg/sec)"
legend("roll","pitch","yaw")

# save figure
savefig("RPY", label, mnum, 800, 800);

endfunction

function timeHacks(limits, xyzr, c1, c2, color)
  hold on
  offset = (limits(4)-limits(3))/40;
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
##  good = (abs(mroll-center) < tol) || (abs(eroll-center) < tol);
  good = (abs(mroll-center) < tol);
endfunction

function u = unwrapd(angled)
  u = rad2deg(unwrap(deg2rad(angled)));  
endfunction

function savefig(name, label, mnum, width, height)
  fname = sprintf("%s_%d_%s", label, mnum, name);
  disp(sprintf("saving figure: %s", fname));
  print ([fname ".jpg"], sprintf("-S%i,%i", width, height))
  hgsave ([fname ".ofig"]) 
endfunction
