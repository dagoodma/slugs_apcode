startIndex = 1;
endIndex = 2016;
gpsSogIdx = 8; % gps raw int. vel
airDynIdx = 59; % scaled pressure diff 1
rawPitIdx = 116; % raw pressure diff 1
airTempIdx = 60; % scaled pressure temperature
rawTempIdx = 118; % raw pressure temperature
timeStampIdx = 11;

time = (data(1:endIndex, timeStampIdx) - data(1:endIndex,timeStampIdx))*0.001;

% Plot normalized velocity, scaled, and raw pressure
figure
gpsVel = data(startIndex:endIndex, gpsSogIdx);
scaledPress = data(startIndex:endIndex, airDynIdx);
rawPress = data(startIndex:endIndex, rawPitIdx);

plot(time(startIndex:endIndex ), (gpsVel - min(gpsVel))/(max(gpsVel)-min(gpsVel)));
hold on
plot(time(startIndex:endIndex), (scaledPress - min(scaledPress))/(max(scaledPress)-min(scaledPress)), 'r');
plot(time(startIndex:endIndex), (rawPress - min(rawPress))/(max(rawPress)-min(rawPress)), 'g');

hold off
title('Normalized GPS SOG (blue), Norm Dynamic pressure (red), Norm dynamic P Raw');
legend('GPS Vel','Scaled Pressure', 'Raw Pressure');

% Plot un-normalized velocity, scaled, and raw pressure
figure;
gpsVel = data(:, gpsSogIdx);
scaledPress = data(:, airDynIdx);
rawPress = data(:, rawPitIdx);
scaledTemp = data(:, airTempIdx);
rawTemp = data(:, rawTempIdx);


subplot(2,2,1);
plot(time(:), rawPress, 'g');
title('Raw Pressure');
subplot(2,2,2);
plot(time(:), scaledTemp);
title('Scaled Temperature');
subplot(2,2,3);
plot(time(:), rawTemp);
title('Raw Temperature');
subplot(2,2,4);
title('WTF');
plotyy(time(:), scaledTemp, time(:), rawTemp);

pause;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dynPas =  sqrt(2*rawPress/1.225);
GPS_h2Pas =  1.225*(gpsVel.^2)/2;
Raw2Pas = (rawPress-1320.5)*0.81887676;
AirSpd = sqrt(2*Raw2Pas/1.225);
figure
subplot(2,1,1)
plotyy(time(:), gpsVel,time(:),rawPress);
title(' GPS SOG (blue), Dynamic pressure Raw');
subplot(2,1,2)
plotyy(time(:), GPS_h2Pas(:),time(:), (rawPress-1320.5)*0.81887676);
plotyy(time(:), gpsVel, time(:), dynPas(:));
plotyy( time(:),GPS_h2Pas(:),time(:), dynPas(:))
figure
plotyy( time(:),GPS_h2Pas(:),time(:), Raw2Pas(:))
plotyy(time(:), gpsVel,time(:), AirSpd(:));
plotyy( time(:),GPS_h2Pas(:),time(:), rawPress)
title(' GPS 2 Pascal (blue), Dynamic pressure Raw');


figure;
plot(time(: ), gpsVel);
hold on
plot(time(:), scaledPress/10, 'r');
hold off

figure
plotyy(time(:), gpsVel,time(:), scaledPress/10);
figure
plotyy(time(:), gpsVel,time(:), rawPress);