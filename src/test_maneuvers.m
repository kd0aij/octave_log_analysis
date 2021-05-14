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

dt = 1 / 25;
res = [];

state.origin = [39.8420194 -105.2123333 1808];
state.pThresh = 88;
state.noise = 0; # degrees, meters
state.windcomp = 1;

# runway heading: this is the angle from the North axis, positive CW
# for runway number 6: compass heading of 60 degrees
# for AAM east field runway compass heading of 106 degrees
rhdg  = 90; 

# rotation about earth Z to runway heading
r2runway = rotv([0 0 1], deg2rad(90-rhdg));

# m/sec in NED earth frame
wind = [0 5 0]; 

# straight line entry
# center of box (150m in front of pilot), 50m AGL
# NED coordinates
pos = [150 0 -50];

# end first arc at center of box (150m in front of pilot)
state.spd = 30;
T = 3;
radius = 50;
pos(2) = -state.spd * T - radius;
pos = (r2runway * pos')';


# state comprises Euler RPY, ECEF position and speed
# init attitude to straight & level, assuming NED, yaw 0 is North
# and yaw CW from North to runway heading is rhdg
##roll  = 0;
##pitch = 0;
##state.quat = euler2quat(roll, pitch, rhdg);

# list of maneuvers
mlist = {
  struct('setstate', 'windcomp', 'on', 1);
  struct('setstate', 'position', 'x', pos(1), 'y', pos(2), 'z', pos(3));

  # rolling spiral
  struct('setstate', 'attitude', 'roll', 0, 'pitch',  20, 'yaw', rhdg);
  struct('maneuver', 'circle', 'radius', 50, 'arc', -360, 'roll', -360);

  # straight, level, rolling entry
  struct('setstate', 'attitude', 'roll', 0, 'pitch',  0, 'yaw', rhdg);
  struct('maneuver', 'roll', 'T', 3, 'arc', 360);
  # 1/4 inside loop, roll to right KE, vertical, roll left to upright
  struct('maneuver', 'arc', 'radius', 50, 'arc', 90, 'roll', 0);
  struct('maneuver', 'roll', 'T', 1, 'arc', 90);
  struct('maneuver', 'line', 'T', 1);
  struct('maneuver', 'roll', 'T', 1, 'arc', -90);
  # 1/4 outside loop
  struct('maneuver', 'arc', 'radius', 50, 'arc', -90, 'roll', 0);
  # rolling half circle
  struct('maneuver', 'circle', 'radius', 50, 'arc', -180, 'roll', -360);
  # straight, level exit
  struct('maneuver', 'line', 'T', 3);
  
  struct('maneuver', 'arc', 'radius', 50, 'arc', -90, 'roll', 90);
  struct('maneuver', 'arc', 'radius', 50, 'arc', -180, 'roll', -180);
  struct('maneuver', 'line', 'T', 3);
};

then = time;

for idx = 1:length(mlist)
  item = mlist{idx};
  fields = fieldnames(item);
  itype = fields{1};
  stype = getfield(item, fields(1){1});
  fstr = [''];
  for i2 = 2:length(fields)
    fstr = [fstr sprintf(", %s:%0.1f", 
                         fields(i2){1}, getfield(item, fields(i2){1}))];
  endfor
  disp(sprintf("item %i: %s:%s%s", idx, itype, stype, fstr));
  switch itype
    case "setstate"
      switch getfield(item, fields(1){1})
        case "windcomp"
          val = getfield(item, "on");
          state.windcomp = val;
        case "attitude"
          roll = getfield(item, "roll");
          pitch = getfield(item, "pitch");
          yaw = getfield(item, "yaw");
          state.quat = euler2quat(roll, pitch, yaw);
        case "position"
           x = getfield(item, "x");
           y = getfield(item, "y");
           z = getfield(item, "z");
           state.pos = [x y z];
      endswitch
    case "maneuver"
      [state data] = pattern_maneuver(mlist{idx}, dt, state, rhdg, wind);
      res = [res; data];
  endswitch
endfor

##figure(7, 'position', [20, 100, 800, 800])
##xyzr = res(:,27:29);
##Nsamp = size(xyzr)(1);
##plot3Dline(xyzr, 'o');
##axis equal
### plot wing planes at intervals
##hold on
##quat = res(:,17:20);
##[wx, wy, wz] = ellipsoid(0, 0, 0, 0.1, 10, 0.01);
##for idx = 1:10:Nsamp
##  # rotate and translate from body to world
##  R = q2rotm(quat(idx,:));
##  [rx, ry, rz] = rellips(R, wx, wy, wz, xyzr(idx,:));
##  s1 = surf(rx, ry, rz);
##  set(s1,'facealpha',0.2);
##endfor
##grid on
##rotate3d on
##title('full')
##xlabel('East')
##ylabel('North')
##zlabel('Alt')

### find heading by fitting a line to xy data
##X = [ones(length(data),1) data(:,27)];
##[beta sigma] = ols(data(:,28), X)
##ehdg = atan2d(beta(2),1);
##figure(8)
##plot(data(:,27),data(:,28),data(:,27),beta(1)+beta(2)*data(:,27),'o')
##axis equal
##title(sprintf("heading: %5.1f, sigma: %5.1f", ehdg, sigma))

# add timestamp column
Nsamp = length(res);
pts = [0:dt:(Nsamp-1)*dt];
res(1:Nsamp,1) = pts;

disp(sprintf("maneuver generation time: %f", time-then));

fflush (stdout);
##ans = input("plot? ", "s");
##if ans == 'y' || ans == 'Y'  
  # plot
  yawCor = rad2deg(atan2(vectorNorm(wind), state.spd));

  rollTolerance = 10;
  plot_title = sprintf("roll tolerance %d degrees, crosswind : %5.1f deg",
                       rollTolerance, yawCor);
##  plot_maneuver(0, (Nsamp-1)*dt, res, 77, 'test_maneuvers',
##                           state.origin, rollTolerance, 2, rhdg=rhdg, state.pThresh, plot_title);
  plot_maneuver_simplified(0, (Nsamp-1)*dt, res, 77, 'test_maneuvers',
                           state.origin, rollTolerance, 2, rhdg=rhdg, state.pThresh, plot_title);
##endif

endfunction
