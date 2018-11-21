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

pkg load quaternion
pkg load geometry
clear res;
close all;

origin = [39.8420194 -105.2123333 1808];

rollTolerance = 10;
pThresh = 88;
noise = 0; # degrees, meters

### pilotNorth is the direction the pilot is facing:
### for rhdg=16, this is 16 degrees east of North: compass heading 16, yaw=??
##pilotNorth = 25; 

# runway heading: this is the angle from the North axis, positive CW
# for runway number 6: compass heading of 60 degrees
# for AAM east field runway compass heading of 106 degrees
rhdg  = 106; 

wind = [0 0 0]; # m/sec in NED earth frame

dt = 1 / 25;
res = [];

# rotation about earth Z from East to runway heading
r2runway = rotv([0 0 1], deg2rad(rhdg-90));

# parameters
# arc, circle: arc, radius
# line: T
# roll: T, arc
maneuver_list = {
  ['roll', 50, 360, 3];
  ['arc', 50, 90, 3];
  };

# straight line entry
# center of box (150m in front of pilot), 50m AGL
# NED coordinates
state.pos = [150 0 -50];

# state comprises Euler RPY, ECEF position and speed
# init attitude to straight & level, assuming NED, yaw 0 is North
# and yaw CW from North to runway heading is rhdg
roll  = 0;
pitch = 0;
state.quat = euler2quat(roll, pitch, rhdg);

# end entry at center of box (150m in front of pilot)
state.spd = 30;
T = 3;
radius = 50;
state.pos(2) = -state.spd * T - radius;

state.pos = (r2runway' * state.pos')';

# on is the default
pattern_maneuver('wind_comp_on', 0, 0, 0, 0,
                 state, noise, pThresh, origin, rhdg, wind);

then = time;
if 1
# full roll
maneuver = 'circle';
arc = 360;
[state data] = pattern_maneuver(maneuver, radius, arc, T, dt,
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

# find heading by fitting a line to xy data
X = [ones(length(data),1) data(:,27)];
[beta sigma] = ols(data(:,28), X)
ehdg = atan2d(beta(2),1);
figure(8)
plot(data(:,27),data(:,28),data(:,27),beta(1)+beta(2)*data(:,27),'o')
axis equal
title(sprintf("heading: %5.1f, sigma: %5.1f", ehdg, sigma))

# inside 1/4 loop
maneuver = 'arc';
radius = 50;
arc = 90;
[state data] = pattern_maneuver(maneuver, radius, arc, T, dt,
                              state, noise, pThresh, origin, rhdg, wind);
res = [res; data];
endif
# roll to right knife edge on vertical line
T = 1;
maneuver = 'roll';
arc = 90;
[state data] = pattern_maneuver(maneuver, radius, arc, T, dt,
                           state, noise, pThresh, origin, rhdg, wind);
res = [res; data];

# hold knife-edge for 2 seconds
maneuver = 'line';
T = 2;
[state data] = pattern_maneuver(maneuver, radius, arc, T, dt,
                           state, noise, pThresh, origin, rhdg, wind);
res = [res; data];

# roll upright on vertical line
maneuver = 'roll';
T = 1;
arc = -90;
[state data] = pattern_maneuver(maneuver, radius, arc, T, dt,
                           state, noise, pThresh, origin, rhdg, wind);
res = [res; data];

# pull through 1/4 loop to inverted
maneuver = 'arc';
radius = 50;
arc = 90;
[state data] = pattern_maneuver(maneuver, radius, arc, T, dt,
                              state, noise, pThresh, origin, rhdg, wind);
res = [res; data];

# exit with 180 roll to upright
maneuver = 'roll';
T = 3;
arc = -180;
[state data] = pattern_maneuver('roll', radius, arc, T, dt, 
                              state, noise, pThresh, origin, rhdg, wind);
res = [res; data];

##return

# add timestamp column
Nsamp = length(res);
pts = [0:dt:(Nsamp-1)*dt];
res(1:Nsamp,1) = pts;

disp(sprintf("maneuver generation time: %f", time-then));

# plot maneuver
yawCor = rad2deg(atan2(vectorNorm(wind), state.spd));
plot_title = sprintf("roll tolerance %d degrees, crosswind : %5.1f deg",
                     rollTolerance, yawCor);
plot_tseg_color2(0, (Nsamp-1)*dt, res, 77, 'test_maneuvers',
                 origin, rollTolerance, 2, rhdg=rhdg, pThresh, plot_title);

endfunction
