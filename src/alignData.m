function res = alignData(POS, GPS, ATT, IMU, NKF1, NKQ1)
# combine select data into a single dataset
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

ats = ATT.data(:,1);
gts = GPS.data(:,1);
pts = POS.data(:,1);
its = IMU.data(:,1);
nts = NKF1.data(:,1);
qts = NKQ1.data(:,1);

# find common start/end times
# it appears that GPS location is NOT accurate when POS data starts
# so skip the first 30 seconds of data
startTime = 30 + max([ats(1) pts(1) its(1) nts(1) qts(1)])
endTime = min([ats(end) pts(end) its(end) nts(end) qts(end)])

# find the sample intervals
asi = getSampleInterval(ats);
gsi = getSampleInterval(gts)
psi = getSampleInterval(pts);
isi = getSampleInterval(its);
nsi = getSampleInterval(nts);
qsi = getSampleInterval(qts);

# use the largest of ATT,POS,IMU,NKF1,NKQ1 as the common sample interval
csi = max([asi psi isi nsi qsi])

astep = csi/asi;
pstep = csi/psi;
istep = csi/isi;
nstep = csi/nsi;
qstep = csi/qsi;

# find the record numbers for start and end times
[s_ats e_ats, Na] = findStartEnd(ats, csi, astep, startTime, endTime);
[s_pts e_pts, Np] = findStartEnd(pts, csi, pstep, startTime, endTime);
[s_its e_its, Ni] = findStartEnd(its, csi, istep, startTime, endTime);
[s_nts e_nts, Nn] = findStartEnd(nts, csi, nstep, startTime, endTime);
[s_qts e_qts, Nq] = findStartEnd(qts, csi, qstep, startTime, endTime);

Nsamp = min([Na Np Ni Nn Nq])
e_ats = s_ats + Nsamp*astep - 1;
e_pts = s_pts + Nsamp*pstep - 1;
e_its = s_its + Nsamp*istep - 1;
e_nts = s_nts + Nsamp*nstep - 1;
e_qts = s_qts + Nsamp*qstep - 1;

res = zeros(Nsamp, 26);
# timestamp
res(1:Nsamp,1) = pts(s_pts:pstep:e_pts);

# Lat, Lng, Alt
res(1:Nsamp,2:4) = POS.data(s_pts:pstep:e_pts,3:5);

# Roll, Pitch, Yaw
res(1:Nsamp,5:7) = ATT.data(s_ats:astep:e_ats,4:2:8);

# Gyro X/Y/Z,  Acc X/Y/Z
res(1:Nsamp,8:13) = IMU.data(s_its:istep:e_its,3:8);

# VE, VN, VD
res(1:Nsamp,14:16) = NKF1.data(s_nts:nstep:e_nts,6:8);

# Q1-4
res(1:Nsamp,17:20) = NKQ1.data(s_qts:qstep:e_qts,3:6);

# GPS (Lat,Lng,Alt)
# assume GPS sample interval is larger than csi (gstep would be < 1)
# replicate GPS records up to the common rate (csi)
s_gts = findStart(gts, startTime);
i = 1;
j = s_gts;
while (j < length(gts))
  while (gts(j) > res(i,1)) && (i < Nsamp)
    res(i,21:23) = GPS.data(j,8:10);
    i += 1;
  endwhile
  j += 1;
endwhile

# corrected Euler RPY
# handle Euler roll/yaw indeterminacy on vertical lines
# convert quaternion to Euler angles; pitch threshold for vertical is 60 degrees 
avgYaw = hdg2yaw(res(1,7));
vThresh = 60;
for i=1:Nsamp
  t = res(i,1);
  # track average yaw while abs(pitch) < vThresh
  pitch = res(i,6);
  yaw = hdg2yaw(res(i,7));
  if abs(pitch) < vThresh
    avgYaw += .05 * (yaw - avgYaw);
  endif
  [r p y] = attFromQuat(res(i,17:20), avgYaw, vThresh);
  if (t > 385) && (t < 386)
    printf("%5.3f: avgYaw: %5.3f, roll: %5.3f, pitch: %5.3f, yaw: %5.3f\n", t, avgYaw, r, p, y);
  endif
  res(i,24:26) = [r p y];
endfor

endfunction

function dt = getSampleInterval(pts)
  # calculate sample interval
  dt = 0;
  M = 1000;
  dt = pts(M) - pts(1);
  dt = round(dt) / M;
endfunction

function startIndex = findStart(ts, startTime)
  # find the record number for start time
  startIndex = 1;
  max = length(ts);
  while (startIndex < max) && (ts(startIndex) < startTime)
    startIndex += 1;
  endwhile
endfunction
  
function [startIndex endIndex N] = findStartEnd(ts, csi, step, startTime, endTime)
  # find the record numbers for start and end times
  startIndex = 1;
  max = length(ts);
  while (startIndex < max) && (abs(ts(startIndex) - startTime) > csi/2)
    startIndex += 1;
  endwhile

  endIndex = startIndex;
  while (endIndex <= max) && (abs(ts(endIndex) - endTime) > csi/2)
    endIndex += step;
  endwhile
  
  N = 1 + ((endIndex - startIndex) / step);
##  printf("start %d:%f, end %d:%f, step: %d, N: %d\n", startIndex, ts(startIndex), ...
##        endIndex, ts(endIndex), step, N);
endfunction
