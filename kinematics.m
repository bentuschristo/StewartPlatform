function [s] = kinematics(q, dq)
global p1 p2 p3 p4 p5 p6
global p1_s p2_s p3_s p4_s p5_s p6_s
global l1 l2 l3 l4 l5 l6
global s1 s2 s3 s4 s5 s6
global n1 n2 n3 n4 n5 n6
global R RT
global qp1 qp2 qp3 qp4 qp5 qp6
global dqp1 dqp2 dqp3 dqp4 dqp5 dqp6
global n1_skew n2_skew n3_skew n4_skew n5_skew n6_skew
global W_tilde
global rp rb

%% Parameters
x = q(1);
y = q(2);
z = q(3);
phi = q(4);
theta = q(5);
psi = q(6);

dx = dq(1);
dy = dq(2);
dz = dq(3);
dphi = dq(4);
dtheta = dq(5);
dpsi = dq(6);

t = [x; y; z];
vt = [dx; dy; dz];

sigma = deg2rad(15);

qt = [phi; theta; psi];
vqt = [dphi; dtheta; dpsi];

%% Angular Velocity

W = [1 0 -sin(theta); 0 cos(phi) cos(theta)*sin(phi); 0 -sin(phi) cos(theta)*cos(phi)]*[dphi; dtheta; dpsi];
W_tilde = [0 -W(3) W(2); W(3) 0 -W(1); -W(2) W(1) 0];

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

%Skew Symmetric Matrix of P --> For Dynamics
p1_s = [0 -p1(3) p1(2); p1(3) 0 -p1(1); -p1(2) p1(1) 0];
p2_s = [0 -p2(3) p2(2); p2(3) 0 -p2(1); -p2(2) p2(1) 0];
p3_s = [0 -p3(3) p3(2); p3(3) 0 -p3(1); -p3(2) p3(1) 0];
p4_s = [0 -p4(3) p4(2); p4(3) 0 -p4(1); -p4(2) p4(1) 0];
p5_s = [0 -p5(3) p5(2); p5(3) 0 -p5(1); -p5(2) p5(1) 0];
p6_s = [0 -p6(3) p6(2); p6(3) 0 -p6(1); -p6(2) p6(1) 0];

%% Platform Rotation Matrix
Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1]; %yaw
Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)]; %pitch
Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)]; %roll

R = Rz*Ry*Rx; %3D Rotation Matrix
RT = transpose(R);

%% Upper Gimbal Point
qp1 = t + (R*p1);
qp2 = t + (R*p2);
qp3 = t + (R*p3);
qp4 = t + (R*p4);
qp5 = t + (R*p5);
qp6 = t + (R*p6);

%% Upper gimbal point velocity
dqp1 = [eye(3) R*transpose(p1_s)*RT]*dq;
dqp2 = [eye(3) R*transpose(p2_s)*RT]*dq;
dqp3 = [eye(3) R*transpose(p3_s)*RT]*dq;
dqp4 = [eye(3) R*transpose(p4_s)*RT]*dq;
dqp5 = [eye(3) R*transpose(p5_s)*RT]*dq;
dqp6 = [eye(3) R*transpose(p6_s)*RT]*dq;

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

%% Unit Vector
n1 = l1/s1;
n2 = l2/s2;
n3 = l3/s3;
n4 = l4/s4;
n5 = l5/s5;
n6 = l6/s6;

%% Skew Symmetric Matrix of n
n1_skew = [0 -n1(3) n1(2); n1(3) 0 -n1(1); -n1(2) n1(1) 0];
n2_skew = [0 -n2(3) n2(2); n2(3) 0 -n2(1); -n2(2) n2(1) 0];
n3_skew = [0 -n3(3) n3(2); n3(3) 0 -n3(1); -n3(2) n3(1) 0];
n4_skew = [0 -n4(3) n4(2); n4(3) 0 -n4(1); -n4(2) n4(1) 0];
n5_skew = [0 -n5(3) n5(2); n5(3) 0 -n5(1); -n5(2) n5(1) 0];
n6_skew = [0 -n6(3) n6(2); n6(3) 0 -n6(1); -n6(2) n6(1) 0];

%% Legs Velocity Vector
dl1 = transpose(n1)*dqp1;
dl2 = transpose(n2)*dqp1;
dl3 = transpose(n3)*dqp1;
dl4 = transpose(n4)*dqp1;
dl5 = transpose(n5)*dqp1;
dl6 = transpose(n6)*dqp1;

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
plot3(xb, yb, zb, '-o'); %plot base

title('Stewart Platform Simulation');  %judul plot
xlabel('Sumbu X'); %label sumbu x
ylabel('Sumbu Y'); %label sumbu y
zlabel('Sumbu Z'); %label sumbu z

axis([-rb-0.1 rb+0.1, -rb-0.1 rb+0.1, 0 0.6]);
hold on;
fill3(xb, yb, zb, 'c'); %memberi fill warna platform {B}

%plot new coordinate
plot3(x1,y1,z1, '-or', 'LineWidth', 3); %plot kaki ke-1
plot3(x2,y2,z2, '-og', 'LineWidth', 3); %plot kaki ke-2
plot3(x3,y3,z3, '-ob', 'LineWidth', 3); %plot kaki ke-3
plot3(x4,y4,z4, '-oc', 'LineWidth', 3); %plot kaki ke-4
plot3(x5,y5,z5, '-om', 'LineWidth', 3); %plot kaki ke-5
plot3(x6,y6,z6, '-ok', 'LineWidth', 3); %plot kaki ke-6

plot3(x_new, y_new, z_new, '-o'); %plot platform

fill3(x_new, y_new, z_new, 'm'); %memberi fill warna platform {P}
grid on;