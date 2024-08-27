%% Stewart Platform Kinematic Simulation
% Made by Benedictus Christo Geroda Cinun

clc
clear variables
close all

syms x(t) y(t) z(t) phi(t) theta(t) psi(t)

%% Parameters
dt = 0.01; %step
t_max = 5; %max time
t = 0:dt:t_max; %iteration

rb = 0.2; %radius frame {B} in meter
rp = 0.16; %radius frame {P} in meter
sigma = deg2rad(15);

%% Desired Position
x = 0;
y = 0;
z = 0.37;

%% Desired Orientation
phi = deg2rad(5*sin(0.02*t*100)); %pitch
theta = deg2rad(5*cos(0.02*t*100)); %roll
psi = 0; %yaw

%% Empty array to store values
s1 = [];
s2 = []; 
s3 = []; 
s4 = []; 
s5 = []; 

%% Make a video of the simulation
v = VideoWriter('KineSim.mp4','MPEG-4');
open(v);

%% Loop
for i = 1:length(t)
    %if the position or the orientation changes , use the letter and add i
                                %(example: x(i))
    %if the position or the orientation doesn't change, just use the letter
                                %(example: x)
    q = [x; y; z; phi(i); theta(i); psi]; 
    
    [s] = kinematics(rp, rb, sigma, q);
    
    s1(i) = s(1);
    s2(i) = s(2);
    s3(i) = s(3);
    s4(i) = s(4);
    s5(i) = s(5);
    s6(i) = s(6);
    
    frame = getframe(gcf);
    writeVideo(v, frame);
end
close(v);

%% Plotting Legs length
figure(2)
plot(t, s1, '.','Color',[0 0 0]);
title("1st Leg's Length");
xlabel('Time (t)');
ylabel('Length (m)');
axis([0 t_max, 0.3 0.55]);
grid on
hold on

figure(3)
plot(t, s2, '.','Color',[0 0 0]);
title("2nd Leg's Length");
xlabel('Time (t)');
ylabel('Length (m)');
axis([0 t_max, 0.3 0.55]);
grid on
hold on

figure(4)
plot(t, s3, '.','Color',[0 0 0]);
title("3rd Leg's Length");
xlabel('Time (t)');
ylabel('Length (m)');
axis([0 t_max, 0.3 0.55]);
grid on
hold on

figure(5)
plot(t, s4, '.','Color',[0 0 0]);
title("4th Leg's Length");
xlabel('Time (t)');
ylabel('Length (m)');
axis([0 t_max, 0.3 0.55]);
grid on
hold on

figure(6)
plot(t, s5, '.','Color',[0 0 0]);
title("5th Leg's Length");
xlabel('Time (t)');
ylabel('Length (m)');
axis([0 t_max, 0.3 0.55]);
grid on
hold on

figure(7)
plot(t, s6, '.','Color',[0 0 0]);
title("6th Leg's Length");
xlabel('Time (t)');
ylabel('Length (m)');
axis([0 t_max, 0.3 0.55]);
grid on
hold on