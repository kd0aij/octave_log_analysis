function [state data] = pattern_maneuver(maneuver, radius, angle, T, dt, state, 
                                         noise, pThresh, origin, rhdg, wind)
  # state comprises attitude quaternion, ECEF position and speed in a structure
  # rhdg is desired ground heading
  # and wind is a velocity vector in earth frame 
  pkg load quaternion;
  pkg load geometry;
  global do_wind_comp = 1;
  
##  [roll pitch yaw] = quat2euler(state.quat);
##  disp(sprintf("maneuver: %10s RPY: %5.1f, %5.1f, %5.1f, T: %5.1f, r: %5.1f, a: %5.1f", 
##                          maneuver, roll, pitch, yaw, T, radius, angle));
  disp(sprintf("maneuver: %10s T: %5.1f, r: %5.1f, a: %5.1f wind_comp: %i", 
                          maneuver, T, radius, angle, do_wind_comp));
  switch (maneuver)
    case 'wind_comp_on'
      do_wind_comp = 1;
    case 'wind_comp_off'
      do_wind_comp = 0;
     
    # Correcting for a crosswind demonstrates problems with Euler
    # angle representation; the solution may be to back out the crosswind-induced
    # yaw angle.  That should transform the attitude to the zero-crosswind case
    # which has no significant problems except for the roll/yaw ambiguity on verticals.
    # A better approach for estimating the yaw value (heading) for resolving
    # the ambiguity would be to fit a plane to the loop (or arc), and use its
    # orientation as the correct yaw angle.  This requires segmenting the gps
    # track into arcs and estimating the plane containing each arc. 
    # Need a parametric representation of a 3D arc: p(s) = p0 + [fx(s) fy(s) fz(s)]
    # For a circle in the x-z plane, we have (dtheta = s/R)
    # p0 = center, fx=R*cos(s/R), fy=p0y, fz=R*sin(s/R)
    # For a circle in an arbitrary plane defined by p0 and orthogonal vectors hdg,v:
    # (one vector will be the heading and vertical v will ideally be Earth Z)
    # f(s) = p0 + R * cos(s/R) * hdg + R * sin(s/R) * v
    # so there are 3 vector parameters for a general arc: p0, hdg, v
    # plus the scalar radius R; hdg and v each represent a single scalar unknown
    # for a total of 6 independent variables: p0, heading, tilt and radius
    # Given a series of points, p_i, projecting them onto the x-y plane will
    # allow a 2D line fit to specify heading. Projecting onto the y-z plane will
    # similarly allow finding the tilt. Next step is projecting the series onto
    # the plane determined by heading and tilt. In this plane, 
    
    # inside loop has a positive angle, outside loop has a negative angle
    case 'arc'
     
      arclen = radius * deg2rad(abs(angle));
      [quatc s_factor] = wind_correctionE(state, wind);
      T = round(arclen / (s_factor*state.spd) / dt) * dt;

      Nsamp = round(T / dt);
      data = zeros(Nsamp, 29);
      
      # loop center (in x-z plane) is determined by initial orientation and position
      # project body-frame -Z axis to arc center
      ctr = radius * hamilton_product(state.quat, [0 0 -1]) + state.pos;
      
      
      # speed spd in m/sec
      # gy in rad/sec (pitch rate)
      gy = deg2rad(angle) / T;
      dtheta = deg2rad(angle) / Nsamp;

      gx = 0;
      gz = 0;
      
      dquat = rot2q([0 1 0], -dtheta);
      dbx = [1 0 0] * state.spd;
        
      for index = 1:Nsamp
        # write current state
        data = writeRes(data, index, noise, pThresh, origin, rhdg,
                        state, gx, gy, gz, quatc);
                 
        # update position by converting velocity to earth frame and adding wind
        dp = (hamilton_product((quatc*state.quat), dbx) + wind) * dt;
        state.pos += dp;
##        disp(sprintf("%5.3f %5.3f %5.3f ", state.pos(1), state.pos(2), state.pos(3)));
        
        # update attitude
        state.quat = unit(state.quat * dquat); # rotate pitch in body frame
##        state.quat = unit(dquat * state.quat); # rotate pitch in earth frame
        # state.quat must represent desired flight path
##        state.quat = unit(quatc * state.quat); 
        # required for wind_correctionE
        [quatc s_factor] = wind_correctionE(state, wind);
        
##        [roll pitch yaw] = quat2euler(state.quat);
##        disp(sprintf("in maneuver: %7s RPY: %5.1f, %5.1f, %5.1f", 
##                                maneuver, roll, pitch, yaw));
      endfor
      # final quaternion determines orientation at exit, except for yaw correction
      
    # straight line with current attitude
    case 'line'
      Nsamp = T / dt;
      data = zeros(Nsamp, 29);

      gx = 0;
      gy = 0;
      gz = 0;
      
      quatc = wind_correctionE(state, wind);
      dp = hamilton_product((quatc*state.quat), [1 0 0] * state.spd * dt) + wind * dt;

      for index = 1:Nsamp
        data = writeRes(data, index, noise, pThresh, origin, rhdg,
                 state, gx, gy, gz, quatc);

        state.pos += dp;
      endfor
        
    # straight line roll through degrees specified by angle
    case 'roll'
      Nsamp = T / dt;
      data = zeros(Nsamp, 29);

      quatc = wind_correctionE(state, wind);
      dtheta = deg2rad(angle) / Nsamp;
      dquat = rot2q([1 0 0], dtheta);

      gx = rad2deg(dtheta) / dt;
      gy = 0;
      gz = 0;
      
      for index = 1:Nsamp
        data = writeRes(data, index, noise, pThresh, origin, rhdg,
                 state, gx, gy, gz, quatc);
                 
        dp = hamilton_product((quatc*state.quat), [1 0 0] * state.spd * dt) + wind * dt;
        state.pos += dp;
        
        # update attitude
        state.quat = unit(state.quat * dquat); # roll in body frame

##        [roll pitch yaw] = quat2euler(state.quat);
##        disp(sprintf("in maneuver: %7s RPY: %5.1f, %5.1f, %5.1f", 
##                                maneuver, roll, pitch, yaw));
      endfor
        
    otherwise
      disp("usage: testEulerAngles(maneuver)");
      disp("where maneuver is one of:")
      disp("arc, line, roll");
      return
    
  endswitch
##  [roll pitch yaw] = quat2euler(state.quat);
##  disp(sprintf("end maneuver: %6s RPY: %5.1f, %5.1f, %5.1f", 
##                          maneuver, roll, pitch, yaw));
endfunction

function [quatc s_factor] = wind_correctionE(state, wind)
  global do_wind_comp;
  
  quatc = quaternion(1);
  s_factor = 1;
  if do_wind_comp
    # desired flightpath in earth frame
    ex = hamilton_product(state.quat, [1 0 0] * state.spd);
    # wind in earth frame
    # uncorrected flightpath
    exw = ex + wind;
    # axis of required rotation is perp to plane containing wind and des. path
    axis = cross(exw, ex);
    # angle of rotation
    angle = asin(vectorNorm(axis) / (vectorNorm(exw) * state.spd));
    s_factor = vectorNorm(ex) / vectorNorm(exw);
    if (angle > 1e-9)
      axis /= vectorNorm(axis);
      quatc = rot2q(axis, angle);
##      disp(sprintf("angle: %5.3f axis:[%5.3f %5.3f %5.3f], s_factor: %5.3f", 
##                    angle, axis(1), axis(2), axis(3), s_factor));
    endif
  endif
endfunction

function [roll pitch r2hzp] = maneuver_roll_pitch(rhdg, quat)
  # given the maneuver heading: rhdg
  # calculate roll angle as angle between rhdg/earthz plane and body x/y plane
  hv = [cosd(rhdg) sind(rhdg) 0];
  
  # this hzplane requires maneuvers to lie in a vertical plane
  hzplane = [-sind(rhdg) cosd(rhdg) 0];
  
  bx = hamilton_product(quat, [1 0 0]);

  # a more general version would allow the maneuver plane to be non-vertical
  # where mplane is (hv cross earthz) rotated about hv by a roll angle
##  hzplane = cross(hv, mplane);  

##  # first rotate body x around heading vector till it's parallel with
##  # hzplane (perpendicular to plane normal hzplane)
##  # project bx into plane normal to hzplane and rotate in this plane
##  bparhv = cross(hv, (cross(bx, hv)));
####  bprphv = dot(bx, hv);
##  
##  # (R(hv,theta) * bx) cross hzplane = 0
##  theta = acos(vectorNorm(bparhv));  
##  r2hzp = rot2q(hv, theta);
##  
##  by = hamilton_product(quat, [0 1 0]);

  # the wind correction angle (WCA) relative to flight path is the
  # angle between body frame x and hzplane
  # This should be independent of roll and pitch: roll does not affect the direction
  # of bx and pitch is a rotation about hzplane, which does not change the angle
  wca_axis = cross(bx, hzplane);
  wca = 90 - asind(vectorNorm(wca_axis));
  
  # to back out wca, rotate about axis cross(bx, hzplane) 
  wca_axis = wca_axis / vectorNorm(wca_axis);
  # this will be inv(quatc) if correct
  r2hzp = rot2q(wca_axis, deg2rad(real(wca)));
  # this is the attitude with body x aligned to maneuver heading
  fquat = unit(r2hzp * quat);
  bxchz = cross(hamilton_product(fquat, [1 0 0]), hzplane);

  # roll is zero when plane of wings is perpendicular to maneuver plane
  
  # perpendicular to body x-z plane transformed to earth frame
  xyplane = hamilton_product(fquat, [0 1 0]);
  
  # angle between wing plane and maneuver plane
  # This isn't always the x component
  xy_cross_hz = cross(hzplane, xyplane);
  # try this to get the sign right
  xy_cross_hz = sign(sum(xy_cross_hz)) * vectorNorm(xy_cross_hz);
  
  # angle between wing plane and maneuver plane
  xydothz = dot(xyplane, hzplane);
  
  # this gives a range of [-180, 180] 
  roll = atan2d(xy_cross_hz, xydothz);
  
  [r pitch y] = quat2euler(fquat);
endfunction

function data = writeRes(data, index, noise, pThresh, origin, rhdg,
                  state, gx, gy, gz, quatc)
                  
  persistent avgYaw;
  
  # position in meters
  pnoise = 2 * noise * (rand(1,3) - 0.5);
  data(index,27:29) = state.pos + pnoise;
                
  # gyros (deg/sec)
  data(index,8:10) = [gx gy gz];
  
  # ATT: Euler fixed angles: RPY 
  quat = unit(quatc * state.quat);
  [roll pitch yaw] = quat2euler([quat.w,quat.x,quat.y,quat.z]);
  # add noise
  n_roll = roll + 2 * noise * (rand - 0.5);
  n_pitch = pitch + 0 * rand;
  n_yaw = yaw + 0 * rand;
  
  data(index,5:7) = [n_roll wrappedPitch(n_pitch) n_yaw];

  # synthetic quaternion Q1-4
  data(index,17:20) = [quat.w quat.i quat.j quat.k];

  # corrected Euler RPY
  # handle Euler roll/yaw indeterminacy on vertical lines
  # convert quaternion to Euler angles; pitch threshold for vertical is pThresh 
  if abs(wrappedPitch(pitch)) < pThresh
##    avgYaw += .2 * (yaw - avgYaw);
    avgYaw = yaw;
  endif
  avgYaw = wrap(avgYaw);
  [r p y] = attFromQuat(data(index,17:20), avgYaw, pThresh);
  
  [rollc pitchc r2hzp] = maneuver_roll_pitch(rhdg, quat);
  testq = r2hzp*quatc;
  testc = arg(testq) > 1e-6;
  if testc
    disp("error: r2hzp not inverse of quatc");
    arg(testq)
    testq
    quatc
    r2hzp
  endif

##  # given the maneuver heading: rhdg
##  # calculate roll angle as angle between rhdg/earthz plane and body x/y plane
##  hv = [cosd(rhdg) sind(rhdg) 0];
##  
##  # this hzplane requires maneuvers to lie in a vertical plane
##  hzplane = [-sind(rhdg) cosd(rhdg) 0];
##  
##  bx = hamilton_product(quat, [1 0 0]);
##
##  # a more general version would allow the maneuver plane to be non-vertical
##  # where mplane is (hv cross earthz) rotated about hv by a roll angle
####  hzplane = cross(hv, mplane);  
##
####  # first rotate body x around heading vector till it's parallel with
####  # hzplane (perpendicular to plane normal hzplane)
####  # project bx into plane normal to hzplane and rotate in this plane
####  bparhv = cross(hv, (cross(bx, hv)));
######  bprphv = dot(bx, hv);
####  
####  # (R(hv,theta) * bx) cross hzplane = 0
####  theta = acos(vectorNorm(bparhv));  
####  r2hzp = rot2q(hv, theta);
####  
####  by = hamilton_product(quat, [0 1 0]);
##
##  # the wind correction angle (WCA) relative to flight path is the
##  # angle between body frame x and hzplane
##  # This should be independent of roll and pitch: roll does not affect the direction
##  # of bx and pitch is a rotation about hzplane, which does not change the angle
##  wca_axis = cross(bx, hzplane);
##  wca = 90 - asind(vectorNorm(wca_axis));
##  
##  # to back out wca, rotate about axis cross(bx, hzplane) 
##  wca_axis = wca_axis / vectorNorm(wca_axis);
##  # this will be inv(quatc) if correct
##  r2hzp = rot2q(wca_axis, deg2rad(real(wca)));
##  
##  if arg(r2hzp*quatc) > 1e-9
##    disp("error: r2hzp not inverse of quatc")
##    bx
##    wca_axis
##    r2hzp
##  endif
##
##  bxchz = cross(hamilton_product(unit(r2hzp * quat), [1 0 0]), hzplane);
##
####    # back out wind correction
####    aq = r2hzp * quat;
####    [r p y] = quat2euler(aq)
####    # back out pitch
####    aq = unit(inv(rot2q([0,1,0],deg2rad(p))) * aq);
####    # remaining rotation is roll
####    [axisr, thetar] = q2rot(aq)
####    
####  rollc = rad2deg(thetar);   
##
##  # roll is zero when plane of wings is perpendicular to maneuver plane
##  
##  # perpendicular to body x-z plane transformed to earth frame
##  xyplane = hamilton_product(unit(r2hzp * quat), [0 1 0]);
##  
##  # angle between wing plane and maneuver plane
##  # This isn't always the x component
##  xy_cross_hz = cross(hzplane, xyplane);
##  # try this to get the sign right
##  xy_cross_hz = sign(sum(xy_cross_hz)) * vectorNorm(xy_cross_hz)
##  
####  # man. plane unit normal
####  mplaneN = xy_cross_hz / vectorNorm(xy_cross_hz);
##  # angle between wing plane and maneuver plane
##  xydothz = dot(xyplane, hzplane);
##  
##  # this gives a range of [-180, 180] 
##  rollc = atan2d(xy_cross_hz, xydothz);
##  
  data(index,24:26) = [rollc pitchc y];
  
##  lat = origin(1) + m2dLat(yp);
##  lng = origin(2) + m2dLng(xp, origin(1));
  
  lla = xyz2lla(state.pos, 0, origin);
  
  # POS, GPS
  data(index,2:4) = [lla(1) lla(2) state.pos(3)];
  data(index,21:23) = [lla(1) lla(2) state.pos(3)];
endfunction

