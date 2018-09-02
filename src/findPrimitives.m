function [lines, arcs] = findPrimitives(ATT,POS,NKF1,IMU,...
  vmin,tmin,rateMin,rateMax,lpd=0.95)
  # find straight line segments parameterized by 3D line equation:
  # P = alpha * v * dt + O
  # where P and O are [x,y,z] coord's, v is a 3D unit vector and alpha is
  # speed in m/sec

  # Input POS contains timestamp, latitude, longitude and altitude in meters
  # Input GPS is used to determine when locations in POS are valid
  # Input vmin sets minimum line/arc speed
  # Input tmin sets minimum duration
  # Input rateMin is the minimum body rotation rate which differentiates arcs from lines
  # Input rateMax is the body rotation rate which differentiates snaps/spins from arcs
  
  # convert lat/lon/alt to xyz with x axis parallel to runway
  # AAM east field runway elevation is 1808m
  xyz = lla2xyz(POS, 16*(pi/180), 1808);  
  
  # calculate sample rate in Hertz, and set minimum speed to 5 m/sec
  pts = POS.data(:,1);
  dt = 0;
  M = 1000;
  for i = 1:M
    dt += pts(i+1) - pts(i);
  endfor
  dt = round(dt) / M
  
  Nmin = tmin / dt
  lmin = tmin * vmin
  
  # it appears that GPS location is accurate when POS data starts
  startTime = pts(1);

  # find candidate segments based on speed and gyro rates
  nts = NKF1.data(:,1);
  vel = NKF1.data(:,6:8);
  speed = vecnorm(vel,2,2);
##  yaw = NKF1.data(:,5);
##  uyaw = unwrap(yaw-180,180);
##  dyaw = diff(uyaw);
  
  its = IMU.data(:,1);
  gv = (180/3.1416)*IMU.data(:,3:5);
  gv_mag = vecnorm(gv,2,2);
  
  # lowpass filter the gyro magnitude 
  avg_mag = zeros(length(gv_mag),1);
  avg = gv_mag(1);
  for i = 2:length(gv_mag)
    avg += (1 - lpd) * (gv_mag(i) - avg);
    avg_mag(i) = avg;
  endfor
  
  idt = 0;
  M = 1000;
  for i = 1:M
    idt += its(i+1) - its(i);
  endfor
  idt = round(idt) / M
  istep = dt / idt

  ats = ATT.data(:,1);
  roll = ATT.data(:,4);
  pitch = ATT.data(:,6);
  yaw = ATT.data(:,8);
  
  # find the record numbers for start time
  s_its = 1;
  while (its(s_its+1) < startTime)
    s_its += 1;
  endwhile
  l_its = length(its) - s_its

  s_ats = 1;
  while (ats(s_ats+1) < startTime)
    s_ats += 1;
  endwhile
  l_ats = length(ats) - s_ats

  s_nts = 1;
  while (nts(s_nts+1) < startTime)
    s_nts += 1;
  endwhile
  l_nts = length(nts) - s_nts

  N = min(floor(l_its/istep), l_ats)
  segType = zeros(N,1);
  tBegin = startTime;
  
  steady = 0;
  lines = {};
  segIndex = 0;
  for index = 0:N-1
    if (steady)
      if (!steadyCheck(speed(s_nts+index), vmin, avg_mag(s_its+(istep*index)), rateMin))     
        # vehicle is no longer in steady cruise
        # check segment duration
        beginT = ats(slBegin);
        endT = ats(index);
        duration = endT - beginT;
        #if steady duration longer than threshold
        if (duration > tmin)
          # record this line segment
          segIndex++;
          lines{segIndex} = [beginT, endT, duration, slBegin, index];
          printf("end segment %d: duration: %5.2f\n", segIndex, duration);
          plot_track2(slBegin,index,POS,1);
        endif
        # reset state
        steady = 0;
      endif
    else
      if (steadyCheck(speed(s_nts+index), vmin, avg_mag(s_its+(istep*index)), rateMin))
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
  for index = 0:N-1
    if (arc)
      if (!arcCheck(speed(s_nts+index), vmin, avg_mag(s_its+(istep*index)), rateMin/2, rateMax))     
        # vehicle is no longer in arc
        # check segment duration
        beginT = ats(slBegin);
        endT = ats(index);
        duration = endT - beginT;
        #if arc duration longer than threshold
        if (duration > tmin)
          # record this line segment
          segIndex++;
          arcs(segIndex) = [beginT, endT, duration, slBegin, index];
          printf("end segment %d: duration: %5.2f\n", segIndex, duration);
          plot_track2(slBegin,index,POS,1);
        endif
        # reset state
        arc = 0;
      endif
    else
      if (arcCheck(speed(s_nts+index), vmin, avg_mag(s_its+(istep*index)), rateMin, rateMax))
        # vehicle is now in arc; remember index into input data arrays
        slBegin = index;
        # set state
        arc = 1;
        #printf("start segment: speed: %5.2f, avg_mag %5.2f\n", speed(s_nts+index), avg_mag(s_its+(istep*index)));
      endif
    endif
  endfor
  
  printf("%d lines, %d arcs\n",length(lines), length(arcs));
  for i = 1:length(lines)
    for j = lines{i}(4):lines{i}(5)
      segType(j) = 1;
    endfor
  endfor
  for i = 1:length(arcs)
    for j = arcs{i}(4):arcs{i}(5)
      segType(j) += 2;
    endfor
  endfor


  figure(2)
  plot(pts,segType,nts,speed,its,avg_mag)  
  legend('type','speed','rate')
  
endfunction

function retval = steadyCheck(speed, speedThresh, gv_mag, rateThresh)
    retval = (speed > speedThresh) && (abs(gv_mag) < rateThresh);
endfunction

function retval = arcCheck(speed, speedThresh, gv_mag, rateMin, rateMax)
    retval = (speed > speedThresh) && (abs(gv_mag) > rateMin) && (abs(gv_mag) < rateMax);
endfunction
