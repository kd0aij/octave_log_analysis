## Copyright (C) 2018 markw
## 
## This program is free software: you can redistribute it and/or modify it
## under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
## 
## This program is distributed in the hope that it will be useful, but
## WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
## 
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see
## <https://www.gnu.org/licenses/>.

## -*- texinfo -*- 
## @deftypefn {} {@var{retval} =} segment_maneuvers (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: markw <markw@DESKTOP-AR9SD00>
## Created: 2018-06-18

function segments = segment_maneuvers2 (sl_dur, p_lp, data, ...
  rollThresh=15, vdThresh=5)
# construct Nx2 matrix containing the start and end indices of
# straight-and-level flight segments
# default sl_dur is 3.0
# default p_lp is 0.95
# data contains fields: timestamp, Lat, Lng, Alt, Roll, Pitch, Yaw,  
#                       GyrX, GyrY, GyrZ, AccX, AccY, AccZ, VE, VN, VD
# default rollThresh is 15 degrees
# default  vdThresh is 5 m/sec
# pos is the dataflash log POS record (default 25Hz)
# att is the dataflash log ATT record (default 25Hz)

# return list of timestamps [start, end]

  ts = data(:,1);
  roll = data(:,5);
  pitch = data(:,6);
  yaw = data(:,7);
  vd = data(:,16);

  N = length(data);
  filtRoll = zeros(N,1);
  filtPitch = zeros(N,1);
  snl = zeros(N,1);

  slBegin = 1;
  avgRoll = 0;
  avgvd = 0;
  d = p_lp; # single-pole IIR low pass filter
  sAndL = 1;
  segIndex = 0;
  segments = {};
  beginUtc = 0;
  endUtc = 0;
  duration = 0;
  
  index = 0;
  for indexA = 1:N-1
    index += 1;
    snl(index) = sAndL;
    avgRoll += (1.0 - d) * (roll(indexA) - avgRoll);
    avgvd += (1.0 - d) * (vd(index) - avgvd);
    filtRoll(index) = avgRoll;
    filtvd(index) = avgvd;

    if (sAndL)
      if (!slCheck(avgRoll, rollThresh, avgvd, vdThresh))     
        # vehicle is no longer straight and level
        # check segment duration
        beginT = ts(slBegin);
        endT = ts(index);
        duration = endT - beginT;
        #if sAndL duration longer than threshold
        if (duration > sl_dur)
          # record this straight & level segment
          segIndex++;
          segments(segIndex) = [beginT, endT, duration, slBegin, index];
          printf("end segment %d: duration: %5.2f\n", segIndex, duration);
##          plot_att(slBegin,index,ATT,segIndex);
##          plot_track2(slBegin,index,POS,segIndex+100);
        endif
        # reset state
        sAndL = 0;
      endif
    else
      if (slCheck(avgRoll, rollThresh, avgvd, vdThresh))
        # vehicle is now straight-and-level; remember index into input data arrays
        slBegin = index;
        # set state
        sAndL = 1;
        printf("start segment: roll: %5.2f, vd %5.2f\n", avgRoll, avgvd);
      endif
    endif
  endfor
    
  # record the last segment
  index--;
  beginT = ts(slBegin);
  endT = ts(index);
  duration = endT - beginT;
  segments(++segIndex) = [beginT, endT, duration, slBegin, index];
endfunction

function retval = slCheck(avgRoll, rollThresh, avgvd, vdThresh)
    retval = ((abs(avgRoll) < rollThresh) || (abs(abs(avgRoll)-180) < rollThresh)) && ...
                   (abs(avgvd) < vdThresh);
endfunction

# input is GPS week, msec
function retval = getUTCmsec(utcval)
  retval = (315964800-18) * 1000 + utcval(1) * 1000 + utcval(2);
endfunction