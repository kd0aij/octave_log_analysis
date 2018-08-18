function plot_att(index1, index2, ATT, fignum=10)

startIndex = index1;
endIndex = index2;

ats = ATT.data((startIndex:endIndex),1);
roll = ATT.data((startIndex:endIndex),4);
pitch = ATT.data((startIndex:endIndex),6);
yaw = ATT.data((startIndex:endIndex),8);

figure(fignum)
plot(ats,roll,ats,pitch)
hold
plot(ats(1),roll(1),"*k",ats(1),pitch(1),"*k")

grid on
title (sprintf("roll-pitch, segments %d:%d", index1, index2))
xlabel "time"
ylabel "degrees"
legend("roll","pitch")
