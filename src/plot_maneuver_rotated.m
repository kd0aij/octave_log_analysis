function plot_maneuver_rotated(startTime, endTime, data,
  mnum=0, label='untitled', origin=[39.8420194 -105.2123333 1808], 
  rTol=15, posIndex=2, rhdg=106, whichplots=[0 1 2 3], pThresh=45, mingspd=10,
  plotTitle='',
  plotMplanes=false)
  
# pThresh and mingspd are critical parameters for determination of maneuver heading,
# which must be correct when calling maneuver_roll_pitch
# TODO: on Thomas David's log 100 stall turn, the calculated roll is wrong
# from the transition from upline to downline until the mhdg reverses when
# groundspeed exceeds mingspd?

# rhdg is runway heading: 0 is North, 90 is East
# latitude is converted to Y axis in meters
# longitude is converted to X axis in meters
  
# data contains fields: 
#         1    2    3    4     5      6    7
# timestamp, Lat, Lng, Alt, Roll, Pitch, Yaw,  

#    8     9    10    11    12    13  14  15  16
# GyrX, GyrY, GyrZ, AccX, AccY, AccZ, VE, VN, VD

# 17:20, 21, 22, 23
# Q1-4

#  21, 22, 23
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
r2box = rhdg - 90;
xyzr = lla2xyz([lat lon z], r2box, origin);

# assign colors representing roll angle
# roll range is (-180,180] degrees
roll  = zeros(Nsamp, 1);
pitch = zeros(Nsamp, 1);
wca   = zeros(Nsamp, 1);
xwnd  = zeros(Nsamp, 1);

e_roll  = data(startIndex:endIndex,5);
e_pitch = data(startIndex:endIndex,6);
e_yaw   = data(startIndex:endIndex,7);
# convert hdg from [0,360] (positive CW) to 
# [180,-180] (positive CW) (with North at 0)
yaw = hdg2yaw(e_yaw);


# calculate average speed
spd = zeros(Nsamp, 1);
for idx = 1:Nsamp
  spd(idx) = norm(vENU(idx, 1:2));
endfor
avgspd = mean(spd);

# calculate ground heading qualified by a minimum groundspeed: mingspd
# retain previous heading while speed is below mingspd
ghdg = zeros(Nsamp, 1);
ghdg(1) = rhdg;
for idx = 2:Nsamp
  if spd(idx) > mingspd
    ghdg(idx) = atan2d(vNED(idx,2),vNED(idx,1));
  else
    ghdg(idx) = ghdg(idx-1);
  endif  
endfor

# compute roll/pitch in maneuver plane
mhdg = zeros(Nsamp, 1);
mplanes = [];
disp(sprintf("Runway heading (CW from North) is %5.1f (East) / %5.1f (West)", 
             rhdg, wrap180(180+rhdg)));

# diagnostics
reverse = zeros(Nsamp, 1);
wca_axis = zeros(Nsamp, 3);

onVertical = 0;
lowgs = false;

then = time;
hyst = 1; # total hysteresis of 1 degree and 1 m/sec
for idx = 2:Nsamp
  # determine maneuver heading based on whether this is a vertical line
  if onVertical
    # maneuver heading is current mplane heading
    ## lock in the maneuver plane till this line is exited
    mhdg(idx) = mplane.hdg;
    # check for exit from vertical line
    if (abs(e_pitch(idx)) < (pThresh - hyst))
      onVertical = 0;
      # on exit from vertical line
      # use ground heading to define maneuver plane
      disp("exit from vertical line")
      # set maneuver plane heading to current ground heading
      mplane.hdg = ghdg(idx);
      mplane.pos = xyzr(idx,:);
      mplane.entry = false;
      disp(sprintf("rotated maneuver heading %3.0f, ghdg: %3.0f", mplane.hdg, ghdg(idx)));
      # record ground heading maneuver plane
      mplanes = setManeuverPlane(tsp(idx), mplanes, mplane, e_pitch(idx), vENU(idx,:), wca(idx));
      mhdg(idx) = ghdg(idx);
    endif
  else
    # maneuver heading is just ground heading
    mhdg(idx) = ghdg(idx);
    # entering vertical line if pitch > threshold or groundspeed is low
    if (abs(e_pitch(idx)) > (pThresh))
      onVertical = 1;
      # on entry to vertical line:
      disp("entry to vertical line")
      # pick aerobatic box heading using previous ground heading
      mplane.hdg = getManeuverPlane(rhdg, ghdg(idx));
      mplane.pos = xyzr(idx,:);
      mplane.entry = true;
      mhdg(idx) = mplane.hdg;
      disp(sprintf("rotated maneuver heading %3.0f, ghdg: %3.0f", mplane.hdg, ghdg(idx)));
      # record vertical maneuver plane
      mplanes = setManeuverPlane(tsp(idx), mplanes, mplane, e_pitch(idx), vENU(idx,:), wca(idx));
    endif  
  endif
    [roll(idx) pitch(idx) wca(idx) wca_axis(idx,:) reverse(idx,:)] = maneuver_roll_pitch(mhdg(idx), quat(idx,:));
    if abs(wca(idx)) > 12
      disp(sprintf("large wca: %5.1f", wca(idx)));
    endif
    # crosswind is ~ |vENU|*sin(wca): so percentage of earthframe velocity is:
    xwnd(idx) = 100 * abs(sind(wca(idx)));
endfor
disp(sprintf("maneuver_roll_pitch elapsed time: %f", time-then));

colors = ones(length(roll),3);
red = hsv2rgb([0,1,1]);
yellow = hsv2rgb([.13,1,1]);
green = hsv2rgb([.25,1,1]);
greeni = hsv2rgb([.25,1,.8]);
blue = hsv2rgb([.60,1,1]);
magenta = [1,0,1];
rollErr = [];

for idx = 1:length(colors)
  if abs(roll(idx)) < rTol
    # level
    colors(idx,:) = green;
  elseif abs(abs(roll(idx))-180) < rTol
    # inverted
    colors(idx,:) = blue;
  elseif abs(abs(roll(idx))-90) < rTol
    # knife edge
    colors(idx,:) = yellow;
  else
    colors(idx,:) = red;
    rollErr = [rollErr idx];
  endif
endfor
sizes = 4 * ones(length(colors),1);

fignum = mnum * 10;
if any(whichplots == 0)
  figure(fignum, 'position', [900,100,800,800])
  clf
  scatter3(xyzr(:,1), xyzr(:,2), xyzr(:,3), 8, colors);
  axis equal
  grid on
  rotate3d on
  title('ENU xyz')
  xlabel('East')
  ylabel('North')
  zlabel('Alt')

  if plotMplanes
    # plot maneuver plane: vertical, rotated to maneuver heading
    hold on
    limits=axis();
    x = [0; 0];
    y = [-25; 25];
    #TODO: assign colors to maneuver planes: how to vectorize this?
    cmat = zeros(2,2,3);
    for idx = 1:length(mplanes)
      # rotate to heading
      theta = 180 - mplanes(idx).hdg + r2box;
      xyr = [(cosd(theta)*x).-(sind(theta)*y), (sind(theta)*x).+(cosd(theta)*y)];
      # translate to vehicle position
      xyr(:,1:2) += mplanes(idx).pos(1:2);
      [xx yy] = meshgrid(xyr(:,1),xyr(:,2));
      zz = [-25 -25; 25 25] + mplanes(idx).pos(3);
      # red for vertical entry, green for exit
      if mplanes(idx).entry
        pcolor = red;
      else
        pcolor = green;
      endif
      for i = 1:3
        cmat(:,:,i) = pcolor(i);
      endfor
      s1 = surf(xx, yy', zz, cmat);
      set(s1, 'facealpha', 0.1);
    endfor
  endif

  # quaternion to rotate to runway heading
  q2runway = rot2q([0 0 1], deg2rad(-r2box));

  # plot delta wing planes every iconInterval seconds
  hold on
  dt = tsp(2) - tsp(1);
  iconInterval = 1;
  icon_ds = round(iconInterval/dt);
  scale = 10;
  gN = 3;
  # 6 xy grid points at [-.1, -1] [.1, -1] [.3, 0] [.1 1] [-.1 1] [-.1 0]
  wx = [-.1 -.1 -.1;
        .1   .3  .1];
  wy = [-1 0 1;
        -1 0 1];
  xx = zeros(2,3);
  yy = zeros(2,3);
  zz = zeros(2,3);
  cmat = zeros(2,3,3);
  for idx = 2:icon_ds:Nsamp
    # rotate and translate from body to world NED
    q = quaternion(quat(idx,1), quat(idx,2), quat(idx,3), quat(idx,4));
    q = q2runway * q;
    R = q2rotm(q);
    for n = 1:3
      for m = 1:2
        v = R * (scale * [wx(m,n); wy(m,n); 0]);
        xx(m,n) =  v(2) + xyzr(idx,1);
        yy(m,n) =  v(1) + xyzr(idx,2);
        zz(m,n) = -v(3) + xyzr(idx,3);
        cmat(m,n,:) = colors(idx,:);
      end
    end
    s1 = surf(xx, yy, zz, cmat);
  endfor

  # add tail fins at intervals
  hold on
  scale = 10;
  x = [-0.1 0.3;
       -0.1 0.3];
  z = [0 0;
       0 0];
  y = [0 0;
       -0.3 0];
  xx = zeros(2,2);
  yy = zeros(2,2);
  zz = zeros(2,2);
  cmat = zeros(2,2,3);
  for idx = 1:icon_ds:Nsamp
    # rotate and translate from body to world NED
    q = quaternion(quat(idx,1), quat(idx,2), quat(idx,3), quat(idx,4));
    q = q * rot2q([1 0 0], pi/2);
    q = q2runway * q;
    R = q2rotm(q);
    for n = 1:2
      for m = 1:2
        v = R * (scale * [x(m,n); y(m,n); z(m,n)]);
        xx(m,n) =  v(2) + xyzr(idx,1);
        yy(m,n) =  v(1) + xyzr(idx,2);
        zz(m,n) = -v(3) + xyzr(idx,3);
        cmat(m,n,:) = colors(idx,:);
      end
    end
    s1 = surf(xx, yy, zz, cmat);
  endfor

  # save figure
  savefig("3D", label, mnum, 800, 800);
endif

if any(whichplots == 1)
  figure(fignum+1, 'position', [400,50,1080,400])
  clf
  hold on
  plot(tsp,  yaw, 'og', "markersize", 3,
  tsp, e_pitch, 'ob', "markersize", 3,
  tsp, wrap180(unwrapd(e_roll), rTol), 'oc', "markersize", 3)
  plot(tsp, mhdg, '-m', tsp, pitch, '-k', tsp, wrap180(unwrapd(roll), rTol), '-r')
  plot(tsp, xwnd, '-k')
  title('Euler RPY vs. maneuver RPY')
  legend(['eyaw'; 'epitch'; 'eroll'; 'mhdg'; 'pitch'; 'roll'; 'xwnd %'],
         "location","northeastoutside")
  axis tight
  grid minor
  # save figure
  savefig("eulerVmp", label, mnum, 1080, .375*1080);
  #fname = sprintf("%s_eulerVmp_%d", label, mnum);
  #disp(sprintf("saving 3D track display: %s", fname));
  #print ([fname ".jpg"], "-S1080,540")
  #hgsave ([fname ".jpg"])
endif

# plot 5 second time intervals
thacks = [];
for idx = 1:125:Nsamp
  thacks = [thacks tsp(idx)];
endfor

if any(whichplots == 2)
  if length(plotTitle) == 0
    plotTitle = sprintf("roll tolerance: %2.0f", rTol);
  endif

  figure(fignum+2, 'position', [100,100,1600,800])
  clf
  subplot(2,2,1)
  scatterPlot(xyzr, 1, 2, sizes, colors, blue, [-350 350 0 250])
  title (sprintf("Top view\n%s", plotTitle))
  xlabel "east (m)"
  ylabel "north (m)"

  subplot(2,2,2)
  scatterPlot(xyzr, 1, 3, sizes, colors, blue, [-350 350 0 400])
  title (sprintf("Judge view"))
  xlabel "east (m)"
  ylabel "alt (m)"

  subplot(2,2,3)
  scatterPlot(xyzr, 2, 3, sizes, colors, blue, [0 300 0 400])
  title (sprintf("End View"))
  xlabel "north (m)"
  ylabel "alt (m)"

  subplot(2,2,4)
  plot(tsp, roll, '-r', tsp, pitch, '-k', tsp, yaw, '-m');
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
  legend("roll","pitch","yaw","location","northeastoutside")

  # save figure
  savefig("maneuver", label, mnum, 800, 800);
endif

if any(whichplots == 3)
  uroll = unwrapd(roll);
  wuroll = wrap180(uroll, rTol);

  figure(fignum+3, 'position', [100,100,1200,900])
  clf
  subplot(2,1,1)
  ax = plotyy(tsp, wuroll, tsp, [pitch mhdg]);
  limits=axis();
##  rline = findobj(ax(1),'linestyle','-');
##  set(rline,"linestyle",'-');
##  set(rline,"marker",'.');
##  set(rline,"markersize",10);
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
##  if length(rollErr) > 0
##    plot(ax(1), tsp(rollErr), roll(rollErr), '.r');
##  endif
  legend("roll", "pitch", "hdg", "location", "northeastoutside");

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

  ### rate plot
  subplot(2,1,2)
  ax = plotyy(tsp,rad2deg(gv),tsp,roll)
  limits=axis();
  axis(ax(1), [tsp(1) tsp(end) -180-rTol 180+rTol]);
  axis(ax(2), [tsp(1) tsp(end) -180-rTol 180+rTol]);
  set(ax(2),'xgrid','off')

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
  xticks(ax(1), thacks);
  grid(ax(1), 'on')

  title ("roll, pitch, yaw rates")
  xlabel "time"
  ylabel(ax(1), "deg/sec")
  ylabel(ax(2), "roll angle")
  legend("roll","pitch","yaw","mroll","location","northeastoutside")

  # save figure
  savefig("RPY", label, mnum, 1000, 750);
endif

# save workspace
save([label ".work"])

endfunction

function timeHacks(limits, xyzr, c1, c2, color)
  hold on
  offset = (limits(4)-limits(3))/40;
  hackIndex = 0;
  for idx = 1:125:length(xyzr)
    scatter(xyzr(idx,c1),xyzr(idx,c2),20,color)
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
  print ([fname ".svg"], sprintf("-S%i,%i", width, height))
  hgsave ([fname ".ofig"]) 
endfunction

function mhdg = getManeuverPlane(rhdg, ghdg)
      # calculate maneuver plane
      # constrain hdg to box or cross-box
      # cross-box heading
      revhdg = wrap180(rhdg+180);
      chdg = wrap180(rhdg+90);
      if abs(ghdg-rhdg) < 45
        mhdg = rhdg;
      elseif abs(ghdg - revhdg) < 45
        mhdg = revhdg;
      elseif abs(ghdg-chdg) < 45
        mhdg = chdg;
      else  
        mhdg = wrap180(180+chdg);
      endif
endfunction

function mplanes = setManeuverPlane(t, mplanes, mplane, pitch, vel3d, wca)
      # record maneuver plane 
      mplanes = [mplanes mplane];     
      disp(sprintf("t: %5.1f pitch: %3.1f, course: %4.1f, gspd: %3.1f, vspd: %3.1f, wca: %3.1f",
              t, pitch, mplane.hdg, 
              vectorNorm(vel3d(1:2)), vectorNorm(vel3d(3)),
              wca));

endfunction
