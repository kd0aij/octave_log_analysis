pkg load quaternion
rhdg = 30; # runway heading

origin = [0 0 0]';
pos = [1 0 0]';
# rotation about earth Z from East to runway heading
rE2runway = rotv([0 0 1], deg2rad(-rhdg));
pos2 = rE2runway * pos;

data = [origin  pos]'
data2 = [origin  pos2]'

figure(3)
plot(data(:,1), data(:,2),'-o',data2(:,1), data2(:,2),'-o');
axis equal

pos3 = rE2runway' * pos2;
data3 = [origin  pos3]'

figure(4)
plot(data2(:,1), data2(:,2),'-o',data3(:,1), data3(:,2),'-o');
axis equal
