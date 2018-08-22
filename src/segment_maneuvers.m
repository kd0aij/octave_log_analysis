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

function segments = segment_maneuvers (sl_dur, p_lp, ATT, NKF1, POS, ...
  rollThresh=15, vzThresh=5)
# construct Nx2 matrix containing the start and end indices of
# straight-and-level flight segments
# default sl_dur is 3.0
# default p_lp is 0.95
# default rollThresh is 15 degrees
# default  vzThresh is 5 m/sec
# pos is the dataflash log POS record (default 25Hz)
# att is the dataflash log ATT record (default 25Hz)

# return list of timestamps [start, end]

ats = ATT.data(:,1);
roll = ATT.data(:,4);
pitch = ATT.data(:,6);
yaw = ATT.data(:,8);

nts = NKF1.data(:,1);

# assume sample rate is the same for ATT and NKF1, but starting time is later for NKF1
# find first matching timestamps
if (ats(1) < nts(1))
  aStart = 2;
  while (ats(aStart) < nts(1))
    aStart += 1;
  endwhile
else
  print("error: NKF1 start time earlier than ATT\n")
  return
endif
    
vz = NKF1.data(:,8);

N = length(ats);
filtRoll = zeros(N,1);
filtPitch = zeros(N,1);
snl = zeros(N,1);

  slBegin = 1;
  avgRoll = 0;
  avgVz = 0;
  d = p_lp; # single-pole IIR low pass filter
  sAndL = 1;
  segIndex = 0;
  segments = {};
  beginUtc = 0;
  endUtc = 0;
  duration = 0;
  
  index = 0;
  for indexA = aStart:N-1
    index += 1;
    snl(index) = sAndL;
    avgRoll += (1.0 - d) * (roll(indexA) - avgRoll);
    avgVz += (1.0 - d) * (vz(index) - avgVz);
    filtRoll(index) = avgRoll;
    filtVz(index) = avgVz;

    if (sAndL)
      if (!slCheck(avgRoll, rollThresh, avgVz, vzThresh))     
        # vehicle is no longer straight and level
        # check segment duration
        beginT = ats(slBegin);
        endT = ats(index);
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
      if (slCheck(avgRoll, rollThresh, avgVz, vzThresh))
        # vehicle is now straight-and-level; remember index into input data arrays
        slBegin = index;
        # set state
        sAndL = 1;
        printf("start segment: roll: %5.2f, vz %5.2f\n", avgRoll, avgVz);
      endif
    endif
  endfor
    
  # record the last segment
  index--;
  beginT = ats(slBegin);
  endT = ats(index);
  duration = endT - beginT;
  segments(++segIndex) = [beginT, endT, duration, slBegin, index];
endfunction

function retval = slCheck(avgRoll, rollThresh, avgVz, vzThresh)
    retval = ((abs(avgRoll) < rollThresh) || (abs(abs(avgRoll)-180) < rollThresh)) && ...
                   (abs(avgVz) < vzThresh);
endfunction

# input is GPS week, msec
function retval = getUTCmsec(utcval)
  retval = (315964800-18) * 1000 + utcval(1) * 1000 + utcval(2);
endfunction