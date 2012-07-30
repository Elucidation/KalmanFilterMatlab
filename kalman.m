% Constants
g = -9.81; % m/s^2
n = 100; % number of points in trajectory

% Ball init conditions
p0 = 1; % m
v0 = 55; % m/s

% Time
t0 = 0; % s
%tf = (-v0-sqrt(v0*v0-4*0.5*g*p0))/(2*0.5*g); % s
tf = 10;
t = linspace(t0,tf,n);

% Actual Trajectory
pf = p0 + v0*t + 0.5*g*t.*t;

rng('default'); % For repeatability
stdev = 2; % Standard deviation of noise (normal distribution mean=0)
noisy = pf + stdev*randn(1,n);

% Kalman filter
% TODO


% Plotting
plot(t,pf,'b',t,noisy,'k');
legend('Actual','Noisy','Predicted');