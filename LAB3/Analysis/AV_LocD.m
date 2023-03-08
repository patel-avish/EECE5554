clc;
clear;
path = "C:\Users\AVISH\OneDrive\Desktop\Data_Lab3\LocationD\vectornav.csv";
[time,x,y,z] = extractdata(path);
fs = 40; % the data is sampled at 40Hz

% Plot gyro x,y,z vs time
figure();
sgtitle("LocationD Gyro x, y, z vs. Time (Fig 5)");
subplot(2,2,1);
plotvstime(x,time)
title("Gyro-X vs time");
subplot(2,2,2);
plotvstime(y,time)
title("Gyro-Y vs time");
subplot(2,2,3);
plotvstime(z,time)
title("Gyro-Z vs time");

% GYRO-X allan plot
mx = averagingfactor(x,fs);
[avarX, tauX] = allanvar(x,mx, fs);
adevX = sqrt(avarX);

[Nx,lineNx,tauNx] = anglerandomwalk(tauX,adevX);
[Kx,lineKx,tauKx] = raterandomwalk(tauX,adevX);
[Bx,scfBx,lineBx,tauBx] = biasinstability(tauX,adevX);


tauParamsX = [tauNx, tauKx, tauBx];
paramsX = [Nx, Kx, scfBx*Bx];
linesX = [lineNx,lineKx,lineBx];


% GYRO-Y allan plot
my = averagingfactor(y,fs);
[avarY, tauY] = allanvar(y,my, fs);
adevY = sqrt(avarY);

[Ny,lineNy,tauNy] = anglerandomwalk(tauY,adevY);
[Ky,lineKy,tauKy] = raterandomwalk(tauY,adevY);
[By,scfBy,lineBy,tauBy] = biasinstability(tauY,adevY);


tauParamsY = [tauNy, tauKy, tauBy];
paramsY = [Ny, Ky, scfBy*By];
linesY = [lineNy,lineKy,lineBy];


% GYRO-Z allan plot
mz = averagingfactor(z,fs);
[avarZ, tauZ] = allanvar(z,mz, fs);
adevZ = sqrt(avarZ);

[Nz,lineNz,tauNz] = anglerandomwalk(tauZ,adevZ);
[Kz,lineKz,tauKz] = raterandomwalk(tauZ,adevZ);
[Bz,scfBz,lineBz,tauBz] = biasinstability(tauZ,adevZ);


tauParamsZ = [tauNz, tauKz, tauBz];
paramsZ = [Nz, Kz, scfBz*Bz];
linesZ = [lineNz,lineKz,lineBz];


% Plot gyro x,y,z vs time
figure();
sgtitle("LocationD Allan variance plots for gyro x, y, z (Fig 4)");
subplot(2,2,1);
plotallan(tauX,adevX,tauParamsX,paramsX,linesX)
title('Allan Deviation with Noise Parameters - GYRO X')
subplot(2,2,2);
plotallan(tauY,adevY,tauParamsY,paramsY,linesY)
title('Allan Deviation with Noise Parameters - GYRO Y')
subplot(2,2,3);
plotallan(tauZ,adevZ,tauParamsZ,paramsZ,linesZ)
title('Allan Deviation with Noise Parameters - GYRO Z')


function [time,xg,yg,zg]=extractdata(path)
%function to extract time,gyro_x,gyro_y_gyro_z from the csv data file
d = readtable(path,VariableNamingRule="preserve");
header_sec = d.("header.stamp.secs")-min(d.("header.stamp.secs"));
header_nsec = d.("header.stamp.nsecs")/(10^9); %convert nano seconds to secs
time = header_sec+header_nsec;
datastr = split(d.data,",");
xg = str2double(datastr(:,11)); % gyro in x direction
yg = str2double(datastr(:,12)); % gyro in y direction
zg_data = split(datastr(:,13),"*");
zg = str2double(zg_data(:,1)); % gyro in z direction
end

function plotvstime(omega,time)
% function to plot angluar rate (gyro) vs time
plot(time,omega)
xlabel('time (sec)')
ylabel('angluar rate (rad/sec)')
xlim([0 max(time)])
end

function m = averagingfactor(omega,Fs)
% function to find (m) Averaging factor, specified as a scalar or vector with ascending integer
% values less than (N-1)/2, where N is the number of samples in Omega
t0 = 1/Fs;
theta = cumsum(omega, 1)*t0;
maxNumM = 100;
L = size(theta, 1);
maxM = 2.^floor(log2(L/2));

m = logspace(log10(1), log10(maxM), maxNumM).';
m = ceil(m); % m must be an integer.
m = unique(m); % Remove duplicates.
end

function [N,lineN,tauN] = anglerandomwalk(tau,adev)
% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = -0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the angle random walk coefficient from the line.
tauN = 1;
logN = slope*log(1) + b;
N = 10^logN;
lineN = N ./ sqrt(tau);
end

function[K,lineK,tauK] = raterandomwalk(tau,adev)
% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = 0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the rate random walk coefficient from the line.
tauK = 3;
logK = slope*log10(3) + b;
K = 10^logK;
lineK = K .* sqrt(tau/3);
end

function[B,scfB,lineB,tauB] = biasinstability(tau,adev)
% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = 0;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the bias instability coefficient from the line.
scfB = sqrt(2*log(2)/pi);
logB = b - log10(scfB);
B = 10^logB;
tauB = tau(i);
lineB = B * scfB * ones(size(tau));
end

function plotallan(tau,adev,tauParams,params,lines)

loglog(tau,adev,'k','linewidth',2)
hold on;
loglog(tau, lines,'--','linewidth',2)
loglog(tauParams,params, 'o','linewidth',2)
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('$\sigma (rad/s)$', '$\sigma_N ((rad/s)/\sqrt{Hz})$', ...
    '$\sigma_K ((rad/s)\sqrt{Hz})$', '$\sigma_B (rad/s)$', 'Interpreter', 'latex')
text(tauParams, params, {'N', 'K', '0.664B'})
grid on
axis equal
hold off;
end
