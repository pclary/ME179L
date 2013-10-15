dist = [485 660 655 545 455 400 350 320 ...
    290 270 245 230 220 205 200 185 180 165 ...
    160 152 145 150 150 145 145 140 140 137 ...
    133 128 120 115 112 112];

fitDist = dist(3:20);
fitDist = 1./fitDist;
x = [3:20]*1e-2;
hold on;
% plot(x,fitDist)
P = polyfit(x,fitDist,1)
plot(x,1./(P(2)+P(1).*x),'r')
plot(x,23.0779./x,'g')
plot(x,dist(3:20),'b')