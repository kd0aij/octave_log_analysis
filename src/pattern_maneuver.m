function [state data] = pattern_maneuver(maneuver, radius, T, dt, state, 
                                         noise, pThresh, origin, rhdg, wind)
  # state comprises attitude quaternion, ECEF position and speed in a structure
  # rhdg is desired ground heading
  # and wind is a velocity vector in earth frame 
  pkg load quaternion;
  pkg load geometry;
  global do_wind_comp = 1;
  
  [roll pitch yaw] = quat2euler(state.quat);
  status_string = sprintf("maneuver: %15s RPY: %5.1f, %5.1f, %5.1f, T: %5.1f", 
                          maneuver, roll, pitch, yaw, T);
  switch (maneuver)
    case 'wind_comp_on'
      do_wind_comp = 1;
    case 'wind_comp_off'
      do_wind_comp = 0;
     
    # Correcting for a crosswind, demonstrates problems with Euler
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
    
    case 'arc'
      status_string = [status_string sprintf(", radius: %5.1f", radius)];
      
      # loop center (in x-z plane) is determined by initial orientation and position
      # project body-frame -Z axis to arc center
      ctr = radius * hamilton_product(state.quat, [0 0 -1]) + state.pos;
      
      # speed spd in m/sec
      # gy in rad/sec (pitch rate)
      gy = state.spd / radius;
      dtheta = gy * dt;

      Nsamp = 1 + T / dt;
      data = zeros(Nsamp, 29);
      gx = 0;
      gz = 0;
      
      dquat = rot2q([0 1 0], -dtheta);
      quatc = wind_correctionE(state, wind);
        
      for i = 1:Nsamp
        # write current state
        data = writeRes(data, i, noise, pThresh, origin, rhdg,
                        state, gx, gy, gz, quatc);
                 
        # update position by converting velocity to earth frame and adding wind
        dp = hamilton_product((quatc*state.quat), [1 0 0] * state.spd * dt) + wind * dt;
        state.pos += dp;
        
        # update attitude
        state.quat = unit(state.quat * dquat); # rotate pitch in body frame
        # state.quat must represent desired flight path
##        state.quat = unit(quatc * state.quat); 
        # required for wind_correctionE
        quatc = wind_correctionE(state, wind);
      endfor
      # final quaternion determines orientation at exit, except for yaw correction
      
    case 'straight_line'
      # T seconds straight line at current attitude
      Nsamp = 1 + T / dt;
      data = zeros(Nsamp, 29);

      gx = 0;
      gy = 0;
      gz = 0;
      
      quatc = wind_correctionE(state, wind);
      dp = hamilton_product((quatc*state.quat), [1 0 0] * state.spd * dt) + wind * dt;

      for i = 1:Nsamp
        data = writeRes(data, i, noise, pThresh, origin, rhdg,
                 state, gx, gy, gz, quatc);

        state.pos += dp;
      endfor
        
    case 'roll'
      # T seconds straight line roll with degrees specified by radius
      Nsamp = 1 + T / dt;
      data = zeros(Nsamp, 29);

      gx = 0;
      gy = 0;
      gz = 0;
      
      quatc = wind_correctionE(state, wind);
      dtheta = deg2rad(radius) / Nsamp;
      dquat = rot2q([1 0 0], dtheta);

      for i = 1:Nsamp
        data = writeRes(data, i, noise, pThresh, origin, rhdg,
                 state, gx, gy, gz, quatc);
                 
        dp = hamilton_product((quatc*state.quat), [1 0 0] * state.spd * dt) + wind * dt;
        state.pos += dp;
        
        # update attitude
        state.quat = unit(state.quat * dquat); # roll in body frame
        quatc = wind_correctionE(state, wind);
      endfor
        
    otherwise
      disp("usage: testEulerAngles(maneuver)");
      disp("where maneuver is one of:")
      disp("loop, rolling_loop, 4_point_rolling_loop");
      return
    
  endswitch
  disp(status_string);
endfunction

##function quatc = wind_correction(state, wind)
##  do_wind_comp = 1;
##  persistent csign=0;
##  
##  if do_wind_comp
##    # desired earth-frame heading is parallel to rhdg
##    # wind correction to attitude cancels crosswind component        
##    # earth-frame air velocity unit vector
##    avel = hamilton_product(state.quat, [1 0 0]);
##    
##    # crosswind component
##    xwind = vectorNorm(cross(wind, avel));
##    
##    # compute yaw correction required to compensate crosswind
##    # This is a body-frame rotation 
##    yawCor = atan2(xwind, state.spd);
##    if (csign == 0)
##      csign = sign(yawCor);
##    else
##      if (csign != sign(yawCor))
##        yawCor
##        csign = sign(yawCor);
##      endif
##    endif
##    quatc = rot2q([0 0 1], -yawCor);
##  else
##    quatc = quaternion(1);
##  endif
##endfunction

##function quatc = wind_correctionB(state, wind)
##  global do_wind_comp;
##  
##  quatc = quaternion(1);
##  if do_wind_comp
##    # desired flightpath in body frame
##    bx = [1 0 0] * state.spd;
##    # wind in body frame
##    bw = hamilton_product(state.quat, wind);
##    # uncorrected flightpath
##    bxw = bx + bw;
##    # axis of required rotation is perp to plane containing wind and des. path
##    axis = cross(bx, bxw);
##    # angle of rotation
##    angle = asin(vectorNorm(axis) / (vectorNorm(bxw) * state.spd));
##    if (angle > 1e-5)
##      axis /= vectorNorm(axis);
##      quatc = rot2q(axis, -angle);
####      disp(sprintf("angle: %5.3f axis:[%5.3f %5.3f %5.3f]", 
####                    angle, axis(1), axis(2), axis(3)));
##    endif
##  endif
##endfunction

function quatc = wind_correctionE(state, wind)
  global do_wind_comp;
  
  quatc = quaternion(1);
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
    if (angle > 1e-5)
      axis /= vectorNorm(axis);
      quatc = rot2q(axis, angle);
##      disp(sprintf("angle: %5.3f axis:[%5.3f %5.3f %5.3f]", 
##                    angle, axis(1), axis(2), axis(3)));
    endif
  endif
endfunction

function data = writeRes(data, i, noise, pThresh, origin, rhdg,
                  state, gx, gy, gz, quatc)
                  
  persistent avgYaw;
  
  # position in meters
  data(i,27:29) = state.pos;
                
  # gyros (deg/sec)
  data(i,8:10) = [gx gy gz];
  
  # ATT: Euler fixed angles: RPY 
  quat = unit(quatc * state.quat);
  [roll pitch yaw] = quat2euler([quat.w,quat.x,quat.y,quat.z]);
  # add noise
  n_roll = roll + 2 * noise * (rand - 0.5);
  n_pitch = pitch + 0 * rand;
  n_yaw = yaw + 0 * rand;
  
  data(i,5:7) = [n_roll wrappedPitch(n_pitch) n_yaw];

  # synthetic quaternion Q1-4
  data(i,17:20) = [quat.w quat.i quat.j quat.k];

  # corrected Euler RPY
  # handle Euler roll/yaw indeterminacy on vertical lines
  # convert quaternion to Euler angles; pitch threshold for vertical is pThresh 
  if abs(wrappedPitch(pitch)) < pThresh
##    avgYaw += .2 * (yaw - avgYaw);
    avgYaw = yaw;
  endif
  avgYaw = wrap(avgYaw);
  [r p y] = attFromQuat(data(i,17:20), avgYaw, pThresh);
  data(i,24:26) = [r p y];
  
##  lat = origin(1) + m2dLat(yp);
##  lng = origin(2) + m2dLng(xp, origin(1));
  
  lla = xyz2lla(state.pos, 0, origin);
  
  # POS, GPS
  data(i,2:4) = [lla(1) lla(2) state.pos(3)];
  data(i,21:23) = [lla(1) lla(2) state.pos(3)];
endfunction

