% Constants
g = 0;%-9.81; % m/s^2
n = 100; % number of points in trajectory

% Ball init conditions
p0 = 0; % m
v0 = 100;%55; % m/s

% Time
t0 = 0; % s
%tf = (-v0-sqrt(v0*v0-4*0.5*g*p0))/(2*0.5*g); % s
tf = 10;
t = linspace(t0,tf,n);
dt = (tf-t0)/n;

% Actual Trajectory
pf = p0 + v0*t + 0.5*g*t.*t;

% Noisy Trajectory
rng('default'); % For repeatability
stdev = 20; % Standard deviation of noise (normal distribution mean=0)
noisy = pf + stdev*randn(1,n);

% Kalman filter

A = [1 dt; 0 1]; % state transition matrix, expected trajectory
B = [dt^2/2; dt]; % input control matrix, expected effect of input accel
C = [1 0]; % measrement matrix, expected measurement

u = g; % acc mag
Q = [p0; v0];
Q_est = Q;
acc_noise_mag = 0.05;
sensor_noise_mag = 10;
Ez = sensor_noise_mag^2;
Ex = acc_noise_mag^2 * [dt^4/4 dt^3/2; dt^3/2 dt^2];
P = Ex; % Estimate of initial position variance


Q_loc_est = [];
vel_est = [];
P_est = P;
P_mag_est = [];
pred_state = [];
pred_var = [];

for i = 1:n
    Q_est = A * Q_est + B * u;
    
    %pred_state = [pred_state; Q_est(1)];
    P = A * P * A' + Ex;
    %pred_var = [pred_var; P];
    K = P*C'*inv(C*P*C'+Ez);
    Q_est = Q_est + K * (noisy(i) - C*Q_est);
    P = ( eye(2) - K*C ) * P;
    
    Q_loc_est = [Q_loc_est; Q_est(1)];
    %vel_est = [vel_est; Q_est(2)];
    %P_mag_est = [P_mag_est; P(1)];
end


% Plotting
plot(t,pf,'r.-',t,noisy,'k.-', t,Q_loc_est,'g.-');
legend('Actual','Noisy','Predicted');