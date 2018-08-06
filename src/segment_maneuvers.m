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

function segments = segment_maneuvers (sl_dur, p_lp, POS, ATT)
# construct Nx2 matrix containing the start and end indices of
# straight-and-level flight segments
# default sl_dur is 3.0
# default p_lp is 0.95
# pos is the dataflash log POS record (default 25Hz)
# att is the dataflash log ATT record (default 25Hz)

# return list of timestamps [start, end]

pts = POS.data(:,1);
lat=POS.data(:,3);
lon=POS.data(:,4);
z = POS.data(:,5);

ats = ATT.data(:,1);
roll = ATT.data(:,4);
pitch = ATT.data(:,6);
yaw = ATT.data(:,8);

N = size(ats)(1);
filtRoll = zeros(N,1);
filtPitch = zeros(N,1);

  slBegin = 1;
  avgRoll = 0;
  avgPitch = 0;
  d = p_lp; # single-pole IIR low pass filter
  rollThresh = 15.0; # 15 degrees
  pitchThresh = 15.0; # 15 degrees
  sAndL = 1;
  segIndex = 0;
  segments = {};
  beginUtc = 0;
  endUtc = 0;
  duration = 0;
  
  for index = 1:size(ats)(1)
    avgRoll += (1.0 - d) * (roll(index) - avgRoll);
    avgPitch += (1.0 - d) * (pitch(index) - avgPitch);
    filtRoll(index) = avgRoll;
    filtPitch(index) = avgPitch;

    if (sAndL)
      if (!slCheck(avgRoll, rollThresh, avgPitch, pitchThresh))     
        # vehicle is no longer straight and level
        # check segment duration
        beginT = ats(slBegin);
        endT = ats(index);
        duration = endT - beginT;
        #if segment longer than threshold
        if (duration > sl_dur)
          # record this segment and start a new one
          segIndex++;
          segments(segIndex) = [beginT, endT, duration, slBegin, index];
        endif
        sAndL = 0;
      endif
    else
      if (!slCheck(avgRoll, rollThresh, avgPitch, pitchThresh))
        # vehicle is now straight-and-level; remember index into input data arrays
        slBegin = index;
        sAndL = 1;
      endif
    endif
  endfor
    
  # record the last segment
  index--;
  beginT = ats(slBegin);
  endT = ats(index);
  duration = endT - beginT;
  segments(++segIndex) = [beginT, endT, duration];
  
  figure();
  plot(ats,filtRoll,ats,filtPitch);

endfunction

function retval = slCheck(avgRoll, rollThresh, avgPitch, pitchThresh)
    retval = ((abs(avgRoll) < rollThresh) || ((abs(avgRoll)-180) < rollThresh)) && ...
                   (abs(avgPitch) < pitchThresh);
endfunction

# input is GPS week, msec
function retval = getUTCmsec(utcval)
  retval = (315964800-18) * 1000 + utcval(1) * 1000 + utcval(2);
endfunction