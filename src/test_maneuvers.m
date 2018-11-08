function test_maneuvers()
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
# 27:29) x, y, z

origin = [39.8420194 -105.2123333 1808];

rollTolerance = 10;
pThresh = 75;
noise = 0; # degrees, meters

rhdg  = 30; # runway heading
wind = [0 5 0]; # m/sec in earth frame

pkg load quaternion

clear res;
close all;

dt = 1 / 25;
res = [];

# rotation about earth Z from East to runway heading
rE2runway = rotv([0 0 1], deg2rad(rhdg));

wind = (rE2runway' * wind')'

# straight line entry
# center of box (150m in front of pilot), 30m AGL
state.pos = [0 150 origin(3)+30];
state.spd = 30;
T = 3;

# init to straight & level
roll  = 0;
pitch = 0;

# state comprises Euler RPY, ECEF position and speed
state.quat = euler2quat(roll, pitch, rhdg);

# end entry at center of box (150m in front of pilot)
state.pos(1) = -state.spd * T;

state.pos = (rE2runway' * state.pos')';

radius = 0;
angle = 0;
pattern_maneuver('wind_comp_on', radius, angle, T, dt,
                 state, noise, pThresh, origin, rhdg, wind);

# roll inverted
angle = 180;
[state data] = pattern_maneuver('roll', radius, angle, T, dt,
                           state, noise, pThresh, origin, rhdg, wind);
res = [res; data];

# find heading by fitting a line to xy data
X = [ones(length(data),1) data(:,27)];
[beta sigma] = ols(data(:,28), X)
ehdg = atan2d(beta(2),1);
figure(8)
plot(data(:,27),data(:,28),data(:,27),beta(1)+beta(2)*data(:,27),'o')
axis equal
title(['heading fit: ' num2str(ehdg)])

# outside 1/4 loop
maneuver = 'arc';
radius = 50;
angle = -90;
[state data] = pattern_maneuver(maneuver, radius, angle, T, dt,
                              state, noise, pThresh, origin, rhdg, wind);
res = [res; data];

# roll to right knife edge on vertical line
T = 1;
angle = 90;
[state data] = pattern_maneuver('roll', radius, angle, T, dt,
                           state, noise, pThresh, origin, rhdg, wind);
res = [res; data];

# hold knife-edge for 2 seconds
T = 2;
[state data] = pattern_maneuver('line', radius, angle, T, dt,
                           state, noise, pThresh, origin, rhdg, wind);
res = [res; data];

# roll back inverted on vertical line
T = 1;
angle = -90;
[state data] = pattern_maneuver('roll', radius, angle, T, dt,
                           state, noise, pThresh, origin, rhdg, wind);
res = [res; data];

# pull through 1/4 loop to straight & level
maneuver = 'arc';
radius = 50;
angle = 90;
[state data] = pattern_maneuver(maneuver, radius, angle, T, dt,
                              state, noise, pThresh, origin, rhdg, wind);
res = [res; data];

# exit with 180 roll to upright
T = 3;
angle = -180;
[state data] = pattern_maneuver('roll', radius, angle, T, dt, 
                              state, noise, pThresh, origin, rhdg, wind);
res = [res; data];

figure(7)
xyzr = res(:,27:29);
plot3Dline(xyzr, 'o');
axis equal
grid on
rotate3d on
title('full')
xlabel('East')
ylabel('North')
zlabel('Alt')

##return

# add timestamp column
Nsamp = length(res);
pts = [0:dt:(Nsamp-1)*dt];
res(1:Nsamp,1) = pts;

# plot maneuver
yawCor = rad2deg(atan2(vectorNorm(wind), state.spd));
plot_title = sprintf("roll tolerance %d degrees, crosswind : %5.1f deg",
                     rollTolerance, yawCor);
plot_tseg_color2(0, (Nsamp-1)*dt, res, 1, 'some_maneuvers',
                 origin, rollTolerance, 2, rhdg, pThresh, plot_title);

endfunction
