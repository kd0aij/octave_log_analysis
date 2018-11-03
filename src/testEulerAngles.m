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

clear res;

origin = [39.8420194 -105.2123333 1808];

dt = 1 / 25;
res = [];

vThresh = 85;
noise = 5; # degrees
rhdg = 20; # runway heading (CCW degrees from East)
radius = 50;

# straight and level entry
rot = [0 0 0];
# center of box (150m in front of pilot), 30m AGL
pos = [0 150 origin(3)+30]';
s = 15;
state = {rot, pos, s};

T = 1;
hdg = rhdg;
# end entry at center of box (150m in front of pilot)
pos(1) = -s * T;

# rotation about earth Z from East to runway heading
rE2runway = rotv([0 0 1], deg2rad(rhdg));
pos = rE2runway' * pos;

[opos hdg data] = do_maneuver('straight_level', radius, T, s, dt,
                              pos, hdg, noise, vThresh, origin);
res = [res; data];

figure(3)
xyzr = lla2xyz(data(:,2:4), 0, origin);
plot(xyzr(:,1), xyzr(:,2), 'o');
axis equal
title('entry')

[opos hdg data] = do_maneuver(maneuver, radius, T, s, dt,
                              opos, hdg, noise, vThresh, origin);
res = [res; data];

figure(4)
xyzr = lla2xyz(data(:,2:4), 0,origin);
plot3Dline(xyzr, 'o');
axis equal
title(maneuver)

# straight and level exit
[opos hdg data] = do_maneuver('straight_level', radius, T, s, dt, 
                              opos, hdg, noise, vThresh, origin);
res = [res; data];

figure(5)
xyzr = lla2xyz(data(:,2:4), 0,origin);
plot(xyzr(:,1), xyzr(:,2), 'o');
axis equal
title('exit')

# add timestamp column
Nsamp = length(res);
pts = [0:dt:(Nsamp-1)*dt];
res(1:Nsamp,1) = pts;

# plot maneuver
plot_tseg_color2(0,(Nsamp-1)*dt,res,1,maneuver,origin,10,2,rhdg)

endfunction
