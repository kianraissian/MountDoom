% Script for climbing mount doom.
% prepare workspace and robot stuff
clear
pub = rospublisher('/raw_vel'); % wheel publisher
msg = rosmessage(pub);

tic
t_elapsed = 0;

sub = rossubscriber('/accel');  % accelerometer subscriber
accelerometer = receive(sub);
acc = accelerometer.Data;

% equations for angle from acceleration reading
syms ax ay az
f_phi = atan2(ay, az);
f_theta = atan2(-ax, sqrt(ay^2 + az^2));

% calculate rotation and movement variables
w = 0.5;    % angular velocity
d = 0.25;   % wheelbase
VL = w*d/2; % right wheel for rotating
VR = -VL;   % left wheel for rotating
V = 0.1;    % forward speed for advancing
phi = 10;

t_rot = 0.1;   % seconds to rotate
t_for = 1;   % second to forward

while abs(phi) > 0.02

% grab subscriber data
accelerometer = receive(sub);   % receive the current accelerometer
a = accelerometer.Data;       % grab the data

% unpack acceleration for substituting
ax = a(1);
ay = a(2);
az = a(3);

phi = double(subs(f_phi))
theta = double(subs(f_theta))

% switch turn depending on orientation of neato
    if phi > 0
        msg.Data = [VR,VL];% rotate clockwise
    else
        msg.Data = [VL,VR];% rotate counterclockwise
    end

send(pub, msg)      
pause(t_rot)       % pause for rotation

msg.Data = [0,0];  % stop after rotating
send(pub, msg)
end

msg.Data = [V,V];
send(pub, msg)
pause(t_for)

msg.Data = [0,0];
send(pub, msg)
t_elapsed = toc;

msg.Data = [0,0];
send(pub, msg)
