pkg load quaternion
rhdg = 30; # runway heading
origin = [39.8420194 -105.2123333 1808];
pos = origin;
pos(2) += m2dLng(1, pos(1));

# construct array with columns [lat lon alt]
data = [origin;  pos]

xyzr = lla2xyz(data, 0, origin);
xyzr2 = lla2xyz(data, 30, origin);

figure(3)
plot(xyzr(:,1), xyzr(:,2),'-o',xyzr2(:,1), xyzr2(:,2),'-o');
axis equal

# test conversion back to lat/lon/alt
pos3 = xyz2lla(xyzr2, -30, origin);
xyzr3 = lla2xyz(pos3, 0, origin);

figure(4)
plot(xyzr2(:,1), xyzr2(:,2),'-o',xyzr3(:,1), xyzr3(:,2),'-o');
axis equal
