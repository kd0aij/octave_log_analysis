function [state data] = pattern_maneuver(mst, dt, istate, rhdg, wind)
  # istate comprises attitude quaternion, ECEF position (meters) and speed in a structure
  # returned state is attitude and position at end of maneuver
  # rhdg is desired ground heading
  # and wind is a velocity vector in earth frame 
  
  # refactoring: various maneuvers require different parameters:
  # arc needs #degrees of arc and radius but T is dependent on arclength and speed
  # line needs only T
  # roll needs T in addition to arc, roll rate is dependent on T and arc
  # a compact list of maneuvers therefore contains variable length commands
  data = [];  
  state = istate;
  origin = state.origin;
  noise = state.noise;
  pThresh = state.pThresh;
  
  switch (mst.maneuver)
     
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
    
    # inside loop has a positive arc, outside loop has a negative arc
    # inputs: arc, radius
    case 'arc'
     
      arclen = mst.radius * deg2rad(abs(mst.arc));
      [quatc s_factor] = wind_correctionE(state, wind);
      T = round(arclen / (s_factor*state.spd) / dt) * dt;

      Nsamp = round(T / dt);
      data = zeros(Nsamp, 29);
      
      # loop center (in x-z plane) is determined by initial orientation and position
      # project body-frame -Z axis to arc center
      ctr = mst.radius * hamilton_product(state.quat, [0 0 -1]) + state.pos;
      
      
      # speed spd in m/sec
      # gy in rad/sec (pitch rate)
      gy = deg2rad(mst.arc) / T;
      dtheta = deg2rad(mst.arc) / Nsamp;

      gx = 0;
      gz = 0;
      
      dquat = rot2q([0 1 0], dtheta);
      dbx = [1 0 0] * state.spd;
        
      for idx = 1:Nsamp
        # write current state
        data = writeRes(data, idx, noise, pThresh, origin, rhdg,
                        state, gx, gy, gz, quatc);
                 
        # update position by converting velocity to earth frame and adding wind
        dp = (hamilton_product((quatc*state.quat), dbx) + wind) * dt;
        state.pos += dp;
##        disp(sprintf("%5.3f %5.3f %5.3f ", state.pos(1), state.pos(2), state.pos(3)));
        
        # update attitude
        state.quat = unit(state.quat * dquat); # rotate pitch in body frame

        # state.quat must represent desired flight path
##        state.quat = unit(quatc * state.quat);
 
        # required for wind_correctionE
        [quatc s_factor] = wind_correctionE(state, wind);
        dquat = rot2q([0 1 0], dtheta/s_factor);
        
##        [roll pitch yaw] = quat2euler(state.quat);
##        disp(sprintf("in maneuver: %7s RPY: %5.1f, %5.1f, %5.1f", 
##                                maneuver, roll, pitch, yaw));
      endfor
      # final quaternion determines orientation at exit, except for yaw correction
      
    # CW (from above) is a positive arc, CCW is a negative arc
    # inputs: arc, radius
    #TODO: set roll angle to match turn rate
    case 'circle'
     
      arclen = mst.radius * deg2rad(abs(mst.arc));
      [quatc s_factor] = wind_correctionE(state, wind);
      T = round(arclen / (s_factor*state.spd) / dt) * dt;

      Nsamp = round(T / dt);
      data = zeros(Nsamp, 29);
      
      # circle center (in x-y plane) is determined by initial orientation and position
      # project body-frame (x-earth z) plane to circle center
      xezp = cross(hamilton_product(state.quat, [1 0 0]), [0 0 1]);
      xezp /= vectorNorm(xezp);
      ctr = sign(arclen) * mst.radius * xezp + state.pos;
      
      
      # speed spd in m/sec
      # gy in rad/sec (pitch rate)
      gy = deg2rad(mst.arc) / T;
      dtheta = deg2rad(mst.arc) / Nsamp;

      gx = 0;
      gz = 0;
      
      dquat = rot2q([0 0 1], dtheta);
      dbx = [1 0 0] * state.spd;
        
      for idx = 1:Nsamp
        # write current state
        data = writeRes(data, idx, noise, pThresh, origin, rhdg,
                        state, gx, gy, gz, quatc);
                 
        # update position by converting velocity to earth frame and adding wind
        dp = (hamilton_product((quatc*state.quat), dbx) + wind) * dt;
        state.pos += dp;
##        disp(sprintf("%5.3f %5.3f %5.3f ", state.pos(1), state.pos(2), state.pos(3)));
        
        # update attitude
        # state.quat must represent desired flight path
        state.quat = unit(dquat * state.quat); # rotate yaw in earth frame
        
        # required for wind_correctionE
        [quatc s_factor] = wind_correctionE(state, wind);
        dquat = rot2q([0 0 1], dtheta/s_factor);
        
##        [roll pitch yaw] = quat2euler(state.quat);
##        disp(sprintf("in maneuver: %7s RPY: %5.1f, %5.1f, %5.1f", 
##                                maneuver, roll, pitch, yaw));
      endfor
      # final quaternion determines orientation at exit, except for yaw correction
      
    # straight line with current attitude
    # inputs: T
    case 'line'
      Nsamp = mst.T / dt;
      data = zeros(Nsamp, 29);

      gx = 0;
      gy = 0;
      gz = 0;
      
      quatc = wind_correctionE(state, wind);
      dp = hamilton_product((quatc*state.quat), [1 0 0] * state.spd * dt) + wind * dt;

      for idx = 1:Nsamp
        data = writeRes(data, idx, noise, pThresh, origin, rhdg,
                 state, gx, gy, gz, quatc);

        state.pos += dp;
      endfor
        
    # straight line roll through degrees specified by arc
    # inputs: T, arc
    case 'roll'
      Nsamp = mst.T / dt;
      data = zeros(Nsamp, 29);

      quatc = wind_correctionE(state, wind);
      dtheta = deg2rad(mst.arc) / Nsamp;
      dquat = rot2q([1 0 0], dtheta);

      gx = rad2deg(dtheta) / dt;
      gy = 0;
      gz = 0;
      
      for idx = 1:Nsamp
        data = writeRes(data, idx, noise, pThresh, origin, rhdg,
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
      disp(["unsupported maneuver: " mst.maneuver]);
      return
    
  endswitch
##  [roll pitch yaw] = quat2euler(state.quat);
##  disp(sprintf("end maneuver: %6s RPY: %5.1f, %5.1f, %5.1f", 
##                          maneuver, roll, pitch, yaw));

endfunction

function [quatc s_factor] = wind_correctionE(state, wind)
  
  quatc = quaternion(1);
  s_factor = 1;
  if state.windcomp
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

function data = writeRes(data, idx, noise, pThresh, origin, rhdg,
                  state, gx, gy, gz, quatc)
                  # pThresh is not used
                  
  persistent avgYaw;
  
  # position in meters (used for plotting in test_maneuvers)
  pnoise = 2 * noise * (rand(1,3) - 0.5);
  data(idx,27:29) = ned2enu(state.pos + pnoise);
                
  # gyros (deg/sec)
  data(idx,8:10) = [gx gy gz];
  
  # ATT: Euler fixed angles: RPY 
  quat = unit(quatc * state.quat);
  [roll pitch yaw] = quat2euler([quat.w,quat.x,quat.y,quat.z]);
  # add noise
  n_roll = roll + 2 * noise * (rand - 0.5);
  n_pitch = pitch + 0 * rand;
  n_yaw = yaw + 0 * rand;
  
  data(idx,5:7) = [n_roll wrappedPitch(n_pitch) n_yaw];
  
  # VN, VE, VD
  data(idx,14:16) = hamilton_product(state.quat, [1 0 0]) * state.spd;

  # synthetic quaternion Q1-4
  data(idx,17:20) = [quat.w quat.i quat.j quat.k];
  
  # convert lat/lon/alt to ENU meters with no rotation
  lla = xyz2lla(ned2enu(state.pos), 0, origin);
  
  # POS, GPS
  data(idx,2:4) = [lla(1) lla(2) lla(3)];
  data(idx,21:23) = [lla(1) lla(2) lla(3)];
endfunction

function enu = ned2enu(ned)
  enu = [ned(2) ned(1), -ned(3)];
endfunction
