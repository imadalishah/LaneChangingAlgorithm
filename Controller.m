clc;    clear all;  close all;
%% Scene Construction
whiteStrip=0;
yellowStrip_U=5;
yellowStrip_D=-5;
axis([0 rdLen -sceneHt sceneHt])
hold on
rectangle('Position',[0,-sceneHt,rdLen,sceneHt-5],'FaceColor',[0.4660 0.6740 0.1880],'EdgeColor',[0.4660 0.6740 0.1880])
rectangle('Position',[0,-5,rdLen,10],'FaceColor',[0 0 0])
rectangle('Position',[0,5,rdLen,sceneHt],'FaceColor',[0.4660 0.6740 0.1880],'EdgeColor',[0.4660 0.6740 0.1880])
rectangle('Position',[rdLen/2+20,-4,60,3],'FaceColor',[1 0 0]) %Rd Block
plot([0 rdLen],[whiteStrip whiteStrip],'--','color','white','LineWidth',2)
plot(pathX(:),pathY(:),'LineWidth',2)
%% Veh Data
v = 10;       	% Initial velocity	 =	10 m/s
L = 3;        	% Wheel Base		 =	3 m
x0 = 0; y0 = 0;   % Vehicle parked as (x,y)=(0,0)
delta = 0;	      % initial steering angle
si = 0;     	% initial heading angle
dt = 1;       	% Time Interval length
iterations = 500;
for t=1:iterations
    % Front Wheeled Mobile Robot (WMR)
    si_k = si + ((v * sin(si) / L) * dt);
    x1 = x0 + (v * cos(delta + si) * dt);
    y1 = y0 + (v * sin(delta + si) * dt);
    if (t==245)	si_k= pi/8;
    end
    if (t==246)	si_k= 0;
    end
    x_mem(1,t)=x1;	y_mem(1,t)=y1;
    x0 = x1;    		y0 = y1;
    si = si_k; 
end
%% Controller Parameters
m = 1140;           % Mass	=	1140 kg
Iz = 1436.24;       % kg*m^2
lf = 1.165;         % CG-to-Front Wheel
lr = 1.165;         % CG-to-Rear Wheel
cf = 155494.663;    % N/rad
cr = 155494.663;    % N/rad
%% State Space Implementation
A = [ 0 1 0 0; 0 (-(cf+cr)/(m*v)) (cf+cr)/m ((lr * cr)-(lf*cf))/m*v; 0 0 0 1; 0 ((lr*cr)-(lf*cf))/(Iz * v) ((lf * cf)-(lr*cr))/Iz -(((lf^2)*cf)+(lr^2)*cr)/(Iz*v)];
B = [0; cf/m; 0; (lf*cf)/Iz];
D_dist = [0; ((lr*cr - lf*cf)/m*v)-v; 0; -(((lf^2)*cf)+(lr^2)*cr)/(Iz*v)]; 
C = [0 0 1 0];
D=[0];
%% Controllability Checking
w=ctrb(A,B);               	% Controllability Matrix
Contollability_Check=rank(w)	% Full Rank Checking
% Poles placement
poles_placement = [-1 -40 -0.5 -5 0];
eigen_A = eig (A)      		% Eigen Values
Null = [0;0;0;0];
A_dot = [A Null; -C 0];	B_dot = [B; 0];	C_dot = [C 0];
dim = [.42 .7 .7 .1];
str = 'STATE FEEDBACK - INTEGRAL CONTROL';
annotation('textbox',dim,'String',str,'FitBoxToText','on');
K_dot=place(A_dot,B_dot,poles_placement);
K = K_dot(:,1:4);		% Feedback Gain
Ki= K_dot(:,5);		% Integral Gain
A_in=[A-B*K -B*Ki; -C 0];
B_in = [0; 0; 0; 0; 1];
C_in = [C 0];
sys2 = ss (A_in, B_in, C_in,D);
%% Tracking of desired path through controller designed
lsim(sys2,y_mem,1:500);
axis([0 500 -30 30]);
