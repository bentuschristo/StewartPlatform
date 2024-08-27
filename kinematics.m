function [s] = kinematics(rp, rb, sigma, q)
%% Parameters
x = q(1);
y = q(2);
z = q(3);
phi = q(4);
theta = q(5);
psi = q(6);

t = [x; y; z];


%% Frame {B}
beta1 = deg2rad(120);
b1_angle = beta1 + sigma;
b6_angle = beta1 - sigma;

beta2 = deg2rad(-120);
b2_angle = beta2 - sigma;
b3_angle = beta2 + sigma;

beta3 = deg2rad(0);
b4_angle = beta3 - sigma;
b5_angle = beta3 + sigma;

b1 = [-rb*sin(b1_angle); rb*cos(b1_angle); 0];
b2 = [-rb*sin(b2_angle); rb*cos(b2_angle); 0];
b3 = [-rb*sin(b3_angle); rb*cos(b3_angle); 0];
b4 = [-rb*sin(b4_angle); rb*cos(b4_angle); 0];
b5 = [-rb*sin(b5_angle); rb*cos(b5_angle); 0];
b6 = [-rb*sin(b6_angle); rb*cos(b6_angle); 0];

%plot bi
xb = [b1(1) b2(1) b3(1) b4(1) b5(1) b6(1) b1(1)];
yb = [b1(2) b2(2) b3(2) b4(2) b5(2) b6(2) b1(2)];
zb = [b1(3) b2(3) b3(3) b4(3) b5(3) b6(3) b1(3)];

%% Frame {P}
gamma1 = deg2rad(180);
p1_angle = gamma1 - sigma;
p2_angle = gamma1 + sigma;

gamma2 = deg2rad(-60);
p3_angle = gamma2 - sigma;
p4_angle = gamma2 + sigma;

gamma3 = deg2rad(60);
p5_angle = gamma3 - sigma;
p6_angle = gamma3 + sigma;

p1 = [-rp*sin(p1_angle); rp*cos(p1_angle); 0];
p2 = [-rp*sin(p2_angle); rp*cos(p2_angle); 0];
p3 = [-rp*sin(p3_angle); rp*cos(p3_angle); 0];
p4 = [-rp*sin(p4_angle); rp*cos(p4_angle); 0];
p5 = [-rp*sin(p5_angle); rp*cos(p5_angle); 0];
p6 = [-rp*sin(p6_angle); rp*cos(p6_angle); 0];


%% Platform Rotation Matrix
Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1]; %yaw
Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)]; %pitch
Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)]; %roll

R = Rz*Ry*Rx; %3D Rotation Matrix

%% Upper Gimbal Point
qp1 = t + (R*p1);
qp2 = t + (R*p2);
qp3 = t + (R*p3);
qp4 = t + (R*p4);
qp5 = t + (R*p5);
qp6 = t + (R*p6);

%% li vector value
l1 = t + (R*p1) - b1; %vektor kaki ke-1
l2 = t + (R*p2) - b2; %vektor kaki ke-2
l3 = t + (R*p3) - b3; %vektor kaki ke-3
l4 = t + (R*p4) - b4; %vektor kaki ke-4
l5 = t + (R*p5) - b5; %vektor kaki ke-5
l6 = t + (R*p6) - b6; %vektor kaki ke-6

%% Leg length (Si)
s1 = sqrt(transpose(l1)*l1); %panjang kaki ke-1
s2 = sqrt(transpose(l2)*l2); %panjang kaki ke-2
s3 = sqrt(transpose(l3)*l3); %panjang kaki ke-3
s4 = sqrt(transpose(l4)*l4); %panjang kaki ke-4
s5 = sqrt(transpose(l5)*l5); %panjang kaki ke-5
s6 = sqrt(transpose(l6)*l6); %panjang kaki ke-6

s = [s1; s2; s3; s4; s5; s6];

%% New platform coordinates
x1 = [b1(1) b1(1)+l1(1)];
x2 = [b2(1) b2(1)+l2(1)];
x3 = [b3(1) b3(1)+l3(1)];
x4 = [b4(1) b4(1)+l4(1)];
x5 = [b5(1) b5(1)+l5(1)];
x6 = [b6(1) b6(1)+l6(1)];

y1 = [b1(2) b1(2)+l1(2)];
y2 = [b2(2) b2(2)+l2(2)];
y3 = [b3(2) b3(2)+l3(2)];
y4 = [b4(2) b4(2)+l4(2)];
y5 = [b5(2) b5(2)+l5(2)];
y6 = [b6(2) b6(2)+l6(2)];

z1 = [b1(3) b1(3)+l1(3)];
z2 = [b2(3) b2(3)+l2(3)];
z3 = [b3(3) b3(3)+l3(3)];
z4 = [b4(3) b4(3)+l4(3)];
z5 = [b5(3) b5(3)+l5(3)];
z6 = [b6(3) b6(3)+l6(3)];

x_new = [b1(1)+l1(1) b2(1)+l2(1) b3(1)+l3(1) b4(1)+l4(1) b5(1)+l5(1) b6(1)+l6(1) b1(1)+l1(1)];
y_new = [b1(2)+l1(2) b2(2)+l2(2) b3(2)+l3(2) b4(2)+l4(2) b5(2)+l5(2) b6(2)+l6(2) b1(2)+l1(2)];
z_new = [b1(3)+l1(3) b2(3)+l2(3) b3(3)+l3(3) b4(3)+l4(3) b5(3)+l5(3) b6(3)+l6(3) b1(3)+l1(3)];

%% Plotting Robot
figure(1)
clf;
plot3(xb, yb, zb, '-o'); %plot the base

title('Stewart Platform Simulation');  %the plot's title
xlabel('X-Axis'); %x axis label
ylabel('Y-Axis'); %y axis label
zlabel('Z-Axis'); %z axis label

axis([-rb-0.1 rb+0.1, -rb-0.1 rb+0.1, 0 0.6]);
hold on;
fill3(xb, yb, zb, 'c'); %give color to the base

%plot new coordinate
plot3(x1,y1,z1, '-or', 'LineWidth', 3); %1st leg's plot
plot3(x2,y2,z2, '-og', 'LineWidth', 3); %2st leg's plot
plot3(x3,y3,z3, '-ob', 'LineWidth', 3); %3st leg's plot
plot3(x4,y4,z4, '-oc', 'LineWidth', 3); %4st leg's plot
plot3(x5,y5,z5, '-om', 'LineWidth', 3); %5st leg's plot
plot3(x6,y6,z6, '-ok', 'LineWidth', 3); %6st leg's plot

plot3(x_new, y_new, z_new, '-o'); %plotting the platform

fill3(x_new, y_new, z_new, 'm'); %give the platform color
grid on;