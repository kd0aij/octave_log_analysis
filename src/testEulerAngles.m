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
#39.8420194 -105.2123333 1808
global clat = 39.8420194;
global clng = -105.2123333;
global alt = 1808;
global avgYaw;
global dt = 1 / 25;
##global data;
res = [];

vThresh = 85;
noise = 5; # degrees
rhdg = 30; # runway heading
radius = 50;

# straight and level entry
s = 15;
T = 1;
hdg = rhdg;
# end entry at center of box (150m in front of pilot)
xo = -s * T;
yo = 150;
x = xo * cosd(hdg) - yo * sind(hdg);
y = xo * sind(hdg) + yo * cosd(hdg);
z = alt + 30;
disp([x y z hdg])
[xp yp zp hdg data] = do_maneuver('straight_level', radius, T, s, dt, x, y, z, hdg, noise, vThresh);
res = [res; data];
disp([xp yp zp hdg])

figure(3)
title('entry')
xyzr = lla2xyz(data(:,2:4), 0,[39.8420194 -105.2123333 1808]);
plot3Dline(xyzr);

[xp yp zp hdg data] = do_maneuver(maneuver, radius, T, s, dt, xp, yp, zp, hdg, noise, vThresh);
res = [res; data];

figure(4)
title(maneuver)
xyzr = lla2xyz(data(:,2:4), 0,[39.8420194 -105.2123333 1808]);
plot3Dline(xyzr);

# straight and level exit
[xp yp zp hdg data] = do_maneuver('straight_level', radius, T, s, dt, xp, yp, zp, hdg, noise, vThresh);
res = [res; data];

figure(5)
title('exit')
xyzr = lla2xyz(data(:,2:4), 0,[39.8420194 -105.2123333 1808]);
plot3Dline(xyzr);

# add timestamp column
Nsamp = length(res);
pts = [0:dt:(Nsamp-1)*dt];
res(1:Nsamp,1) = pts;

# plot maneuver
plot_tseg_color2(0,(Nsamp-1)*dt,res,1,maneuver,...
  [39.8420194 -105.2123333 1808],10,2,-rhdg)

endfunction
