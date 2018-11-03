function [lines, arcs] = findPrimitives(data,...
  vmin,tmin,rateMin,rateMax,lpd=0.95,origin=[39.8420194 -105.2123333 1808],
  pilotNorth=16)
  # find straight line segments parameterized by 3D line equation:
  # P = alpha * v * dt + O
  # where P and O are [x,y,z] coord's, v is a 3D unit vector and alpha is
  # speed in m/sec

  # Input data contains columns:
  # 1)timestamp, 
  # 2:4)Lat, Lng, Alt, 
  # 5:7)Roll, Pitch, Yaw, 
  # 8:10)GyrX, GyrY, GyrZ, 
  # 11:13)AccX, AccY, AccZ, 
  # 14:16)VE, VN, VD, 
  # 17:20)Q1-4, 
  # 21:23)(raw GPS)Lat, Lng, Alt, (sample-replicated from 5Hz to 25Hz)
  # 24:26)(corrected Euler)Roll, Pitch, Yaw
  
  # Input vmin sets minimum line/arc speed
  # Input tmin sets minimum duration
  # Input rateMin is the minimum body rotation rate which differentiates arcs from lines
  # Input rateMax is the body rotation rate which differentiates snaps/spins from arcs
  
  # convert lat/lon/alt to xyz with x axis parallel to runway
  # AAM east field runway elevation is 1808m, pilot station faces 16 degrees
  # east of true north
  POS = data(:,2:4);
  xyz = lla2xyz(POS, pilotNorth, origin);  
  
  pts = data(:,1);
  startTime = pts(1);

  # find candidate segments based on speed and gyro rates
  vel = data(:,14:16);
  speed = vecnorm(vel,2,2);
  
  gv = (180/pi)*data(:,8:10);
  gv_mag = vecnorm(gv,2,2);
  
  # lowpass filter the gyro magnitude 
  avg_mag = zeros(length(gv_mag),1);
  avg = gv_mag(1);
  for i = 2:length(gv_mag)
    avg += (1 - lpd) * (gv_mag(i) - avg);
    avg_mag(i) = avg;
  endfor
  
  roll = data(:,5);
  pitch = data(:,6);
  yaw = data(:,7);
  
  N = length(data);
  segType = zeros(N,1);
  tBegin = startTime;
  
  steady = 0;
  lines = {};
  segIndex = 0;
  for index = 1:N
    if (steady)
      if (!steadyCheck(speed(index), vmin, avg_mag(index), rateMin))     
        # vehicle is no longer in steady cruise
        # check segment duration
        beginT = pts(slBegin);
        endT = pts(index);
        duration = endT - beginT;
        #if steady duration longer than threshold
        if (duration > tmin)
          # record this line segment
          segIndex++;
          lines{segIndex} = [beginT, endT, duration, slBegin, index];
          printf("end segment %d: duration: %5.2f\n", segIndex, duration);
          plot_track4(slBegin,index,xyz,1);
        endif
        # reset state
        steady = 0;
      endif
    else
      if (steadyCheck(speed(index), vmin, avg_mag(index), rateMin))
        # vehicle is now in steady cruise; remember index into input data arrays
        slBegin = index;
        # set state
        steady = 1;
        #printf("start segment: speed: %5.2f, avg_mag %5.2f\n", speed(s_nts+index), avg_mag(s_its+(istep*index)));
      endif
    endif
  endfor

  arc = 0;
  arcs = {};
  segIndex = 0;
  for index = 1:N
    if (arc)
      if (!arcCheck(speed(index), vmin, avg_mag(index), rateMin/2, rateMax))     
        # vehicle is no longer in arc
        # check segment duration
        beginT = pts(slBegin);
        endT = pts(index);
        duration = endT - beginT;
        #if arc duration longer than threshold
        if (duration > tmin)
          # record this line segment
          segIndex++;
          arcs(segIndex) = [beginT, endT, duration, slBegin, index];
          printf("end segment %d: duration: %5.2f\n", segIndex, duration);
          plot_track4(slBegin,index,xyz,1);
        endif
        # reset state
        arc = 0;
      endif
    else
      if (arcCheck(speed(index), vmin, avg_mag(index), rateMin, rateMax))
        # vehicle is now in arc; remember index into input data arrays
        slBegin = index;
        # set state
        arc = 1;
        #printf("start segment: speed: %5.2f, avg_mag %5.2f\n", speed(s_nts+index), avg_mag(s_its+(istep*index)));
      endif
    endif
  endfor
  
  printf("%d lines, %d arcs\n",length(lines), length(arcs));

  # type will be zero if neither steady nor arc, 3 if both
  # snap roll should be type zero
  # "steady cruise" is type 1: set bit 0
  for i = 1:length(lines)
    for j = lines{i}(4):lines{i}(5)
      segType(j) = 1;
    endfor
  endfor
  # set bit two for arc
  for i = 1:length(arcs)
    for j = arcs{i}(4):arcs{i}(5)
      segType(j) += 2;
    endfor
  endfor


  figure(2)
  plot(pts,segType,pts,speed,pts,avg_mag)  
  legend('type','speed','rate')
  
  # fit a 3D line to each line segment
  # P = alpha * v * dt + O
  # O=xyz(lines{i}(4)
  # linsolve A*x=b where b is the Nx3 matrix of xyz(lines{i}(4):lines{i}(5),:)
  # and A is 
  
endfunction

function retval = steadyCheck(speed, speedThresh, gv_mag, rateThresh)
    retval = (speed > speedThresh) && (abs(gv_mag) < rateThresh);
endfunction

function retval = arcCheck(speed, speedThresh, gv_mag, rateMin, rateMax)
    retval = (speed > speedThresh) && (abs(gv_mag) > rateMin) && (abs(gv_mag) < rateMax);
endfunction
