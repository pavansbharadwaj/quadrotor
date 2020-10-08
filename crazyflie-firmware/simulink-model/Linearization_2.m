clc; clear variables; close all; format longG;

% syms m g d k b Jx Jy Jz
m = 0.027;
g = 9.81;
d = 0.046;
k = 1e-9;
b = 2.75e-11;

Jx = 1.395e-5;
Jy = 1.436e-5;
Jz = 2.173e-5;

syms base_thrust

syms ctrl1 ctrl2 ctrl3 ctrl4
ctrl = [ctrl1;ctrl2;ctrl3;ctrl4];

syms px py pz
p = [px; py; pz];

syms vx vy vz
v = [vx;vy;vz];

syms ax ay az
a = [ax;ay;az];

syms roll pitch yaw
angle = [roll;pitch;yaw];

syms wx wy wz
w = [wx;wy;wz];

J = [Jx,0,0;0,Jy,0;0,0,Jz];

syms dwx dwy dwz
dw = [dwx;dwy;dwz];

R_ib3=[sin(pitch);
				-cos(pitch)*sin(roll);
				cos(roll)*cos(pitch)];%rotational matrix

nonlinear = [	vx;
							vy;
							vz;
							0+R_ib3(1)*(ctrl(1)+ctrl(2)+ctrl(3)+ctrl(4))/m;
							0+R_ib3(2)*(ctrl(1)+ctrl(2)+ctrl(3)+ctrl(4))/m;
							-g+R_ib3(3)*(ctrl(1)+ctrl(2)+ctrl(3)+ctrl(4))/m;
							wx;
							wy;
							wz;
							-(Jz*wy*wz - Jy*wy*wz + (2^(1/2)*d*(ctrl1 + ctrl2 - ctrl3 - ctrl4))/2)/Jx;
							-(Jx*wx*wz - Jz*wx*wz + (2^(1/2)*d*(ctrl1 - ctrl2 - ctrl3 + ctrl4))/2)/Jy;
							-((k*(ctrl1 - ctrl2 + ctrl3 - ctrl4))/b - Jx*wx*wy + Jy*wx*wy)/Jz];

states = [p;v;angle;w];
equlibrium_states = [0;0;0; %px0, py0, pz0
					0;0;0; %vx0, vy0, vz0
					0;0;0; %roll0, pitch0, yaw0
					0;0;0]; %wx0, wy0, wz0
% equlibrium_input = [1;1;1;1];
equlibrium_input = [1;1;1;1];

T = 0.01;

A = subs(subs(jacobian(nonlinear,states),states,equlibrium_states),ctrl,equlibrium_input);
B = subs(subs(jacobian(nonlinear,ctrl),states,equlibrium_states),ctrl,equlibrium_input);
A = double(A);
B = double(B);

% important_states = [7,8,10,11];
important_states = [7,8,9,10,11,12];

A = A(important_states,important_states);
B = B(important_states,:);
C = eye(size(A));
D=0;

ss_lin = ss(A,B,C,D);
ss_lin_d = c2d(ss_lin,T);

% Q=eye(length(A(1,:))).*[600 300 100 50];
Q=eye(length(A(1,:))).*[10^4*ones(1,3) 10^3*ones(1,3)];

R=eye(length(B(1,:)));

N = 0;
 % we are supposed to either use lqrd or Dlqr
[K0,~,~] = lqrd(A,B,Q,R,T);
[K1,~,~] = lqrd(ss_lin.A, ss_lin.B,Q,R,T);
[K2,~,~] = dlqr(ss_lin_d.A, ss_lin_d.B ,Q,R,N);

K0-K1
K0-K2
K1-K2


K = K0;
disp("done")

save('LQR_gain.mat', 'K');