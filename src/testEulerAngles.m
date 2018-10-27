function res = testEulerAngles(maneuver)
# generate synthetic alignData(POS, GPS, ATT, IMU, NKF1, NKQ1) results

# 25Hz is the normal logging rate for all but IMU and GPS
# with IMU being 50Hz and GPS only 5Hz

# Inputs:
# field 1 of all records is timestamp in seconds
# POS fields: Lat, Lng, Alt (data(3:5))
# GPS fields: Lat, Lng, Alt (data(8:10))
# ATT fields: Roll, Pitch, Yaw (data(4,6,8))
# IMU fields: GyrX/Y/Z (data(3:5), AccX/Y/Z (data(6:8))
# NKF1 fields: VN, VE, VD, (data(6:8))

# Output columns:
# 1)timestamp, 
# 2:4)Lat, Lng, Alt, 
# 5:7)Roll, Pitch, Yaw, 
# 8:10)GyrX, GyrY, GyrZ, 
# 11:13)AccX, AccY, AccZ, 
# 14:16)VE, VN, VD, 
# 17:20)Q1-4, 
# 21:23)(raw GPS)Lat, Lng, Alt, (sample-replicated from 5Hz to 25Hz)
# 24:26)(corrected Euler)Roll, Pitch, Yaw

pkg load quaternion

clear data res;

global clat = 39.8421572;
global clng = -105.2122285;
global alt = 1808;
global avgYaw;
global dt = 1 / 25;
global data;
res = [];

vThresh = 85;
noise = 5; # degrees
rhdg = 45; # runway heading

# straight and level entry
s = 15;
T = 3;
hdg = rhdg;
xp = -s * T * cosd(hdg);
yp = -s * T * sind(hdg);
zp = alt + 30;
[x y z data] = straight_level(xp, yp, zp, hdg, s, T);
res = [res; data];

radius = 50;
initTheta = -90; # starting point is bottom of CCW loop
switch (maneuver)
  case 'half_loop'
    # speed s in m/sec
    # gy in rad/sec (pitch rate)
    gy = s / radius;
    dtheta = gy * dt;
    Nsamp = pi / dtheta;
    # simulate pass by reference into writeRes
    data = zeros(Nsamp, 26);
    roll = 0;
    gx = 0;
    gz = 0;
    theta = initTheta;
    for i = 1:Nsamp
      pitch = 90 + theta;
      dxy = radius * cosd(theta);
      xp = x + dxy * cosd(hdg);
      yp = y + dxy * sind(hdg);
      zp = z + radius + radius * sin(deg2rad(theta));  
      writeRes(i, noise, vThresh, ...
               roll, pitch, hdg, gx, gy, gz, xp, yp, zp);
      theta += rad2deg(dtheta);
    endfor
    hdg += 180;  # heading is reversed at exit
  case 'loop'
    # speed s in m/sec
    # gy in rad/sec (pitch rate)
    gy = s / radius;
    dtheta = gy * dt;
    Nsamp = 2*pi / dtheta;
    data = zeros(Nsamp, 26);
    roll = 0;
    hdg = 0;
    gx = 0;
    gz = 0;
    yp = y;
    theta = initTheta;
    for i = 1:Nsamp
      pitch = 90 + theta;
      xp = x + radius * cos(deg2rad(theta));
      zp = z + radius + radius * sin(deg2rad(theta));  
      writeRes(i, noise, vThresh, ...
               roll, pitch, hdg, gx, gy, gz, xp, yp, zp);
      theta += rad2deg(dtheta);
    endfor
  case 'rolling_loop'
    # speed s in m/sec
    # gy in rad/sec (pitch rate)
    gy = s / radius;
    dtheta = gy * dt;
    Nsamp = 2*pi / dtheta;
    data = zeros(Nsamp, 26);
    gx = gy;
    gz = 0;
    hdg = 0;
    yp = y;
    theta = initTheta;
    for i = 1:Nsamp
      roll = 90 + theta;
      pitch = 90 + theta;
      xp = x + radius * cos(deg2rad(theta));
      zp = z + radius + radius * sin(deg2rad(theta));    
      writeRes(i, noise, vThresh, ...
               roll, pitch, hdg, gx, gy, gz, xp, yp, zp);
      theta += rad2deg(dtheta);
    endfor
  case '4_point_rolling_loop'
    # speed s in m/sec
    # gy in rad/sec (pitch rate)
    gy = s / radius;
    dtheta = gy * dt;
    Nsamp = 2*pi / dtheta;
    data = zeros(Nsamp, 26);
    hdg = 0;
    gz = 0;
    yp = y;
    theta = initTheta;
    for i = 1:Nsamp
      if i==1
        roll = 0;
        lastroll = roll;
      endif
      # roll at 2gy deg/sec during odd octants
      octant = floor((360+theta+22.5)/45);
      if mod(octant,2) == 1
        gx = 2 * gy;
        roll += rad2deg(gx) * dt;
        lastroll = roll;
      else 
        roll = lastroll;  
        gx = 0;      
      endif
      pitch = 90 + theta;
      xp = x + radius * cos(deg2rad(theta));
      zp = z + radius + radius * sin(deg2rad(theta));    
      writeRes(i, noise, vThresh, ...
               roll, pitch, hdg, gx, gy, gz, xp, yp, zp);
      theta += rad2deg(dtheta);
    endfor
    otherwise
      disp("usage: testEulerAngles(maneuver)");
      disp("where maneuver is one of:")
      disp("loop, rolling_loop, 4_point_rolling_loop");
      return
endswitch
res = [res; data];

# straight and level exit
s = 15;
T = 3;
[x y z data] = straight_level(xp, yp, zp, hdg, s, T);

figure(3)
xyzr = lla2xyz(res(:,2:4), 0,[39.8420194 -105.2123333 1808]);
plot3Dline(xyzr);

res = [res; data];

figure(4)
xyzr = lla2xyz(res(:,2:4), 0,[39.8420194 -105.2123333 1808]);
plot3Dline(xyzr);

# add timestamp column
Nsamp = length(res);
pts = [0:dt:(Nsamp-1)*dt];
res(1:Nsamp,1) = pts;

# plot maneuver
plot_tseg_color2(0,(Nsamp-1)*dt,res,1,maneuver,...
  [39.8420194 -105.2123333 1808],10,2,-rhdg)

endfunction

function data = writeRes(i, noise, vThresh, ...
                  roll, pitch, hdg, gx, gy, gz, xp, yp, zp)
                
  # data is declared externally and modified in this function
  # the other globals are not modified  
  global data clat clng alt avgYaw;
  
  if i == 1
    avgYaw = hdg;
  endif

  n_roll = roll + 2 * noise * (rand - 0.5);
  n_pitch = pitch + 0 * rand;
  n_hdg = hdg + 0 * rand;
  
  # gyros (deg/sec)
  data(i,8:10) = [gx gy gz];
  
  # ATT
  data(i,5:7) = [roll wrappedPitch(pitch) hdg];

  qr = rot2q([1,0,0],deg2rad(n_roll));
  qp = rot2q([0,1,0],deg2rad(n_pitch));
  qy = rot2q([0,0,1],deg2rad(n_hdg));
  quat = qy * qp * qr;

  # Q1-4
  data(i,17:20) = [quat.w quat.i quat.j quat.k];

  # corrected Euler RPY
  # handle Euler roll/yaw indeterminacy on vertical lines
  # convert quaternion to Euler angles; pitch threshold for vertical is 60 degrees 
  if abs(wrappedPitch(pitch)) < vThresh
    avgYaw += .2 * (hdg - avgYaw);
  endif
  [r p y] = attFromQuat(data(i,17:20), avgYaw, vThresh);
  data(i,24:26) = [r p y];
  
  lat = clat + m2dLat(yp);
  lng = clng + m2dLng(xp, clat);
  
  # POS, GPS
  data(i,2:4) = [lat lng zp];
  data(i,21:23) = [lat lng zp];
endfunction

function [x y z data] = straight_level(x, y, z, hdg, s, T)
  global dt;
  
  # T seconds straight and level entry at speed s
  # starting at x,y,z on heading h
  Nsamp = T / dt;
  global data;
  data = zeros(Nsamp, 26);

  roll = 0;
  pitch = 0;
  gx = 0;
  gy = 0;
  gz = 0;
  noise = 0;
  vThresh = 10;
  
  dx = s * dt * cosd(hdg);
  dy = s * dt * sind(hdg);
  
  for i = 1:Nsamp
    writeRes(i, noise, vThresh, ...
             roll, pitch, hdg, gx, gy, gz, x, y, z);
    x += dx;
    y += dy;
  endfor
endfunction
