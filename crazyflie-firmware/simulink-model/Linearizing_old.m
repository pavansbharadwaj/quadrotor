clc; clear variables; close all;
format longG

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
% base_thrust = 30000;

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
equlibrium_input = [1;1;1;1]*30000;
A = subs(subs(jacobian(nonlinear,states),states,equlibrium_states),ctrl,equlibrium_input);
B = subs(subs(jacobian(nonlinear,ctrl),states,equlibrium_states),ctrl,equlibrium_input);
Ad = double(A);
Bd = double(B);
Q=eye(12,12);
R=eye(4,4);
N = 0;
%[K,L]=dare(Ad,Bd,Q,R);
% [K,S,e] = lqr(Ad,Bd,Q,R,N); 
[K,S,e] = lqrd(Ad,Bd,Q,R,N);% we are supposed to either use lqrd or Dlqr

size(Ad);
size(Bd);
size(K);
LQR_Gain = Ad-Bd*K
