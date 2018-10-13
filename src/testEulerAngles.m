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

global clat = 39.8421572;
global clng = -105.2122285;
global alt = 2300;
global avgYaw;
dt = 1 / 25;

roll = 0;
pitch = 0;
yaw = 0;

xp = 0;
yp = 0;
zp  = alt;
radius = 300;

vThresh = 85;
noise = 5; # degrees

initTheta = -90; # starting point is bottom of CCW loop
i = 0;
switch (maneuver)
  case 'loop'
    Nsamp = 360;
    global res = zeros(Nsamp, 26);
    for theta = initTheta:Nsamp+initTheta-1
      i += 1; # i is 1 on first iteration
      roll = 0;
      gx = 0;
      pitch = 90 + theta;
      gy = deg2rad(1) / dt;
      yaw = 40;
      gz = 0;
      xp = radius * cos(deg2rad(theta));
      yp = 0;
      zp = alt + radius * sin(deg2rad(theta));  
      writeRes(i, noise, vThresh, ...
               roll, pitch, yaw, gx, gy, gz, xp, yp, zp);
    endfor
  case 'rolling_loop'
    Nsamp = 360;
    global res = zeros(Nsamp, 26);
    for theta = initTheta:Nsamp+initTheta-1
      i += 1; # i is 1 on first iteration
      roll = 90 + theta;
      gx = deg2rad(1) / dt;
      pitch = 90 + theta;
      gy = deg2rad(1) / dt;
      yaw = 40;
      gz = 0;
      xp = radius * cos(deg2rad(theta));
      yp = 0;
      zp = alt + radius * sin(deg2rad(theta));    
      writeRes(i, noise, vThresh, ...
               roll, pitch, yaw, gx, gy, gz, xp, yp, zp);
    endfor
  case '4_point_rolling_loop'
    Nsamp = 360;
    global res = zeros(Nsamp, 26);
    for theta = initTheta:Nsamp+initTheta-1
      i += 1; # i is 1 on first iteration
      if i==1
        roll = 0;
        lastroll = roll;
      endif
      # roll at 2/dt deg/sec during odd octants
      octant = floor((360+theta+22.5)/45);
      if mod(octant,2) == 1
        roll += 2;
        gx = deg2rad(2) / dt;
        lastroll = roll;
      else 
        roll = lastroll;  
        gx = 0;      
      endif
      pitch = 90 + theta;
      gy = deg2rad(1) / dt;
      yaw = 0;
      gz = 0;
      xp = radius * cos(deg2rad(theta));
      yp = 0;
      zp = alt + radius * sin(deg2rad(theta));    
      writeRes(i, noise, vThresh, ...
               roll, pitch, yaw, gx, gy, gz, xp, yp, zp);
    endfor
endswitch

# timestamp
pts = [0:dt:(Nsamp-1)*dt];
res(1:Nsamp,1) = pts;

plot_tseg_color2(0,(Nsamp-1)*dt,res,1,maneuver,...
  [39.8420194 -105.2123333 1808],10,2,0)

endfunction

function writeRes(i, noise, vThresh, ...
                  roll, pitch, yaw, gx, gy, gz, xp, yp, zp)
                  
  global res clat clng alt avgYaw;
  
  if i == 1
    avgYaw = yaw;
  endif

  n_roll = roll + 2 * noise * (rand - 0.5);
  n_pitch = pitch + 0 * rand;
  n_yaw = yaw + 0 * rand;
  
  # gyros (deg/sec)
  res(i,8:10) = [gx gy gz];
  
  # ATT
  res(i,5:7) = [roll wrappedPitch(pitch) yaw];

  qr = rot2q([1,0,0],deg2rad(n_roll));
  qp = rot2q([0,1,0],deg2rad(n_pitch));
  qy = rot2q([0,0,1],deg2rad(n_yaw));
  quat = qy * qp * qr;

  # Q1-4
  res(i,17:20) = [quat.w quat.i quat.j quat.k];

  # corrected Euler RPY
  # handle Euler roll/yaw indeterminacy on vertical lines
  # convert quaternion to Euler angles; pitch threshold for vertical is 60 degrees 
  if abs(wrappedPitch(pitch)) < vThresh
    avgYaw += .2 * (yaw - avgYaw);
  endif
  [r p y] = attFromQuat(res(i,17:20), avgYaw, vThresh);
  res(i,24:26) = [r p y];
  
  lat = clat + m2dLat(yp);
  lng = clng + m2dLng(xp, clat);
  
  # POS, GPS
  res(i,2:4) = [lat lng zp];
  res(i,21:23) = [lat lng zp];
endfunction