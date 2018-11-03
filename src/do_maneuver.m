function [state data] = do_maneuver(maneuver, radius, T, s, dt, state, 
                                    noise, pThresh, origin)
  # state comprises Euler RPY, ECEF position and speed in a structure
  pkg load quaternion;
  
  roll = state.RPY(1);
  pitch = state.RPY(2);
  yaw = state.RPY(3);
  disp([roll pitch yaw]);
  pos = state.pos;
  s = state.s;
  
  global avgYaw = yaw;
  
  x = pos(1);
  y = pos(2);
  z = pos(3);
  cyaw = cosd(yaw);
  syaw = sind(yaw);

  disp(maneuver);
  switch (maneuver)
    # the logic for all of these maneuvers must be generalized if we allow
    # arbitrary state at entry. Currently, the initial pitch and roll are fixed
    # but in the case of a rolling loop, initial roll could be arbitrary.
    # Initial pitch for a loop could also be made arbitrary by offsetting
    # the theta loop variable; done for half_loop only...
    case 'half_loop'
      # loop center (in x-z plane) is determined by initial pitch and position
      initTheta = pitch - 90; # starting point is bottom of CCW loop
      dxy = radius * cosd(initTheta);
      # rotate to plane specified by yaw angle
      cx = x - dxy * cyaw;
      cy = y - dxy * syaw;
      cz = z - radius * sind(initTheta);
      
      # speed s in m/sec
      # gy in rad/sec (pitch rate)
      gy = s / radius;
      dtheta = gy * dt;
      Nsamp = pi / dtheta;
      data = zeros(Nsamp, 26);
      gx = 0;
      gz = 0;
      xp = x;
      yp = y;
      zp = z;
      quat = euler2quat(roll, -pitch, yaw);
      dquat = rot2q([0,1,0], -dtheta);
      dv = [s*dt 0 0];
      for i = 1:Nsamp
        [roll pitch yaw] = quat2euler([quat.w,quat.x,quat.y,quat.z]);
        # write current state
        data = writeRes(data, i, noise, pThresh, origin,
                 roll, pitch, yaw, gx, gy, gz, xp, yp, zp);
                 
        # update position
        dp = hamilton_product(quat, dv);
        xp += dp(1);
        yp += dp(2);
        zp += dp(3);
        # update attitude
        quat = unit(quat * dquat);
      endfor
      # TODO: compute these from final quaternion
##      roll += 180; # roll flips 180 degrees at exit
##      yaw += 180;  # heading is reversed at exit
      pitch *= -1;
      
    case 'half_outside_loop'
      # need to generalize the state to persist at least R and T
      # Thus, a half outside loop performed after a half inside loop
      # will result in an S shape
      initTheta = pitch + 90; # starting point is top of CCW loop

      # loop center (in x-z plane) is determined by initial pitch and position
      dxy = radius * cosd(initTheta);
      # rotate to plane specified by yaw angle
      cx = x - dxy * cyaw;
      cy = y - dxy * syaw;
      cz = z - radius * sind(initTheta);

      # speed s in m/sec
      # gy in rad/sec (pitch rate)
      gy = -s / radius;
      dtheta = -gy * dt;
      Nsamp = abs(pi / dtheta);
      data = zeros(Nsamp, 26);
      gx = 0;
      gz = 0;
##      theta = initTheta;
      xp = x;
      yp = y;
      zp = z;
      quat = euler2quat(roll, -pitch, yaw);
      dquat = rot2q([0,1,0], -dtheta);
      dv = [s*dt 0 0];
      for i = 1:Nsamp
##        pitch = 90 - theta;
##        dxy = radius * cosd(theta);
##        xp = cx + dxy * cosd(yaw);
##        yp = cy + dxy * sind(yaw);
##        zp = cz + radius * sind(theta);  
        data = writeRes(data, i, noise, pThresh, origin,
                 roll, pitch, yaw, gx, gy, gz, xp, yp, zp);
##        theta += rad2deg(dtheta);
        # update position
        dp = hamilton_product(quat, dv);
        xp += dp(1);
        yp += dp(2);
        zp += dp(3);
        # update attitude
        quat = unit(quat * dquat);
      endfor
      roll += 180; # roll flips 180 degrees at exit
      yaw += 180;  # and heading is reversed at exit
      
    case 'loop'
      initTheta = -90; # starting point is bottom of CCW loop
      # speed s in m/sec
      # gy in rad/sec (pitch rate)
      gy = s / radius;
      dtheta = gy * dt;
      Nsamp = 2*pi / dtheta;
      data = zeros(Nsamp, 26);
      gx = 0;
      gz = 0;
      yp = y;
      theta = initTheta;
      for i = 1:Nsamp
        pitch = 90 + theta;
        dxy = radius * cosd(theta);
        xp = x + dxy * cosd(yaw);
        yp = y + dxy * sind(yaw);
        zp = z + radius + radius * sind(theta);  
        data = writeRes(data, i, noise, pThresh, origin,
                 roll, pitch, yaw, gx, gy, gz, xp, yp, zp);
        theta += rad2deg(dtheta);
      endfor
      
    case 'rolling_loop'
      initTheta = -90; # starting point is bottom of CCW loop
      # speed s in m/sec
      # gy in rad/sec (pitch rate)
      gy = s / radius;
      dtheta = gy * dt;
      Nsamp = 2*pi / dtheta;
      data = zeros(Nsamp, 26);
      gx = gy;
      gz = 0;
      yp = y;
      theta = initTheta;
      for i = 1:Nsamp
        roll = 90 + theta;
        pitch = 90 + theta;
        dxy = radius * cosd(theta);
        xp = x + dxy * cosd(yaw);
        yp = y + dxy * sind(yaw);
        zp = z + radius + radius * sind(theta);    
        data = writeRes(data, i, noise, pThresh, origin,
                 roll, pitch, yaw, gx, gy, gz, xp, yp, zp);
        theta += rad2deg(dtheta);
      endfor
      
    case '4_point_rolling_loop'
      initTheta = -90; # starting point is bottom of CCW loop
      # speed s in m/sec
      # gy in rad/sec (pitch rate)
      gy = s / radius;
      dtheta = gy * dt;
      Nsamp = 2*pi / dtheta;
      data = zeros(Nsamp, 26);
      gz = 0;
      yp = y;
      theta = initTheta;
      for i = 1:Nsamp
        if i==1
          roll = 0;
          lastroll = roll;
        endif
        # roll at 2gy deg/sec during odd octants
        octant = floor((360+theta+22.5)/45);
        if mod(octant,2) == 1
          gx = 2 * gy;
          roll += rad2deg(gx) * dt;
          lastroll = roll;
        else 
          roll = lastroll;  
          gx = 0;      
        endif
        pitch = 90 + theta;
        dxy = radius * cosd(theta);
        xp = x + dxy * cosd(yaw);
        yp = y + dxy * sind(yaw);
        zp = z + radius + radius * sind(theta);    
        data = writeRes(data, i, noise, pThresh, origin,
                 roll, pitch, yaw, gx, gy, gz, xp, yp, zp);
        theta += rad2deg(dtheta);
      endfor
      
    case 'straight_level'
      # T seconds straight and level entry at speed s
      # starting at x,y,z on heading h
      Nsamp = T / dt;
      data = zeros(Nsamp, 26);

      gx = 0;
      gy = 0;
      gz = 0;
      noise = 0;
      
      xp = x;
      yp = y;
      zp = z;
      
      quat = euler2quat(roll, -pitch, yaw);
      dv = [s*dt 0 0];
      dp = hamilton_product(quat, dv);

      for i = 1:Nsamp
        data = writeRes(data, i, noise, pThresh, origin,
                 roll, pitch, yaw, gx, gy, gz, xp, yp, zp);

        xp += dp(1);
        yp += dp(2);
        zp += dp(3);
      endfor
        
    otherwise
      disp("usage: testEulerAngles(maneuver)");
      disp("where maneuver is one of:")
      disp("loop, rolling_loop, 4_point_rolling_loop");
      return
  endswitch
  state.RPY = [roll pitch yaw];
  state.pos = [xp yp zp];
  state.s = s;
endfunction

function data = writeRes(data, i, noise, pThresh, origin,
                  roll, pitch, yaw, gx, gy, gz, xp, yp, zp)
                  
  global avgYaw;
                
  n_roll = roll + 2 * noise * (rand - 0.5);
  n_pitch = pitch + 0 * rand;
  n_yaw = yaw + 0 * rand;
  
  # gyros (deg/sec)
  data(i,8:10) = [gx gy gz];
  
  # ATT
  data(i,5:7) = [roll wrappedPitch(pitch) yaw];

  qr = rot2q([1,0,0],deg2rad(n_roll));
  qp = rot2q([0,1,0],deg2rad(n_pitch));
  qy = rot2q([0,0,1],deg2rad(n_yaw));
  quat = qy * qp * qr;

  # Q1-4
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
  
  lat = origin(1) + m2dLat(yp);
  lng = origin(2) + m2dLng(xp, origin(1));
  
  # POS, GPS
  data(i,2:4) = [lat lng zp];
  data(i,21:23) = [lat lng zp];
endfunction

