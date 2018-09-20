function testEuler(roll)
# test Euler angles near pitch = +/- 90
pkg load quaternion

rpy = zeros(181,3);


i = 1;
pitch = -90;
for yaw = -90:90
  qr = rot2q([1 0 0], deg2rad(roll));
  qp = rot2q([0 1 0], deg2rad(pitch));
  qy = rot2q([0 0 1], deg2rad(yaw));
  q = qy * qp * qr;

##  [roll pitch yaw] = quat2euler([q.w q.i q.j q.k]);
##  printf("1) roll: %5.3f, pitch: %5.3f, yaw: %5.3f\n", roll, pitch, yaw);
##
##
##    # rotate by 90 degrees on pitch (y) axis
##    rot_view = rot2q([0 1 0], -deg2rad(90));
##    q = rot_view * q;
##
##    [roll pitch yaw] = quat2euler([q.w q.i q.j q.k]);  
##    printf("2) roll: %5.3f, pitch: %5.3f, yaw: %5.3f\n", roll, pitch, yaw);
  
  [r p y] = attFromQuat([q.w q.x q.y q.z], yaw);
  rpy(i,:) = [r p y];
  
##  roll += .1;
  i += 1;
endfor

plot(rpy(:,2),rpy(:,1),'o',rpy(:,2),rpy(:,3),'o')
legend('r','y')
xlabel('pitch')
