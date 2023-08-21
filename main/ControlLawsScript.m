
R2D=180/pi; %Rad2Deg conversion
RPM_Conversion=30/pi; %Rad/sec to RPM conversion
%% Drone properties 
m=3.4811482949175;      %Mass [kg]
g=9.80665;              %Gravity [N/kg]
L=0.4; %Arm length (Symetric config.)

Jx=0.082298105960482;   %Inertia [Kg*m^2]
Jy=0.082457201628499;
Jz=0.15887270137511;

k_T=1.34e-5; %Thrust coefficient of propellers
k_y=2.28e-7; %Yaw coefficient of propellers

bp=0.001; %rotating drag coefficient
bq=0.001;
br=0.001;

cx=5; %translation drag coefficient
cy=5;
cz=5;

cdx=cx*m; %Effective damping coefficient
cdy=cy*m;
cdz=cz*m;

ts2=1; %Desired Position settling time
tsz=2; %Desired altitude settling time
ts=ts2/2;  %Desired attitude settling time  
tsy=ts; %Desired yaw settling time



roll_angle_limit=32/R2D; %angle (in Rad) to which the drone can incline
pitch_angle_limit=32/R2D;

max_rpm=10000/RPM_Conversion; %Max rpm of rotors in Rad/s
hov_rpm=6200/RPM_Conversion; %prop speed while hovering in Rad/s
hov_thrust=6*k_T*(hov_rpm)^2; %thrust while hovering
max_thrust=6*k_T*(max_rpm)^2; %Max thrust
max_pitch_moment=sqrt(3)*k_T*L*(max_rpm)^2; %Max vehicle torque
max_roll_moment=2*k_T*L*(max_rpm)^2; 
max_yaw_moment=k_y*hov_thrust/k_T;


%% Attitude ctrl (inner loop 5X faster)


Tc=ts/25; %Time constant  % 5x faster than outer loop response (1s) 

%Roll ctrl
Kq=(Jy/Tc)-bq;   %inner loop derivative ctrl theta dot. Determined from 1st order transfer fcn form 
Kq2=Jy*((4/ts))*((-4/ts)+((bq+Kq)/Jy))/Kq; %Outer loop propportional for attitude ctrl

%Pitch ctrl
Kp=(Jx/Tc)-bp;
Kp2= Jx*((4/ts))*((-4/ts)+((bp+Kp)/Jx))/Kp;

%Yaw ctrl
Kr=(Jz/(tsy/25))-br;
Kr2= Jz*((4/tsy))*((-4/tsy)+((br+Kr)/Jz))/Kr;

%% Position ctrl (with zero placement)

% X ctrl

alpha=-(bq+Kq)/(2*Jy);%for pole placement and computing clarity
beta=((2*alpha)^2)-4*Kq2*Kq/Jy;

p1=(alpha+(sqrt(beta))/2);%poles of 4th order Transfer function
p2=(alpha-(sqrt(beta))/2);
p3=-cx;
p4=0;


t1=atan((4/ts2)/(p1+(4/ts2))); %angles from poles to target poles
t2=atan((4/ts2)/(p2+(4/ts2)));
t3=atan((4/ts2)/(p3+(4/ts2)));
t4=3*pi/4;

z_x=cx; %Zero placement for desired reponse~~Warnings: if zero is placed at the first target pole, we end up having 2 poles and a zero on the same point
                                             %in the real world it might not be as accurate but it will still be stable around the pole 
tz=atan((4/ts2)/(-z_x+(4/ts2))); %angle from zero to target pole

l1=(p1+(4/ts2))/(cos(t1)); %lengths from poles/zero to targets 
l2=(p2+(4/ts2))/(cos(t2));
l3=(p3+(4/ts2))/(cos(t3));
l4=(p4+(4/ts2))/(cos(t4));
lz=(-z_x+(4/ts2))/(cos(tz));


Kx=l1*l2*l3*l4/((Kq*Kq2*g/Jy)*lz); %prod of pole lengths divided prod of zero lengths 
%Kx= (p1+4/ts2)*(p2+4/ts2)*(p4+4/ts2)/(Kq*Kq2*g/Jy);
Kx=abs(Kx);


% %root locus check
Pxs=[p1 p2 p3 p4];
 
 a=cx;%computing simplicity
 b=(bq+Kq)/Jy;
 c=Kq*Kq2/Jy;



%sys = tf([c*g],[1 (a+b) (a*b+c) (a*c) 0]);
% X RL w/ zero
sys = tf([c*g z_x*c*g],[1 (a+b) (a*b+c) (a*c) 0]);
% X RL w zero
%rlocus(sys) 

% Y ctrl

alpha_y=-(bp+Kp)/(2*Jx);
beta_y=((2*alpha_y)^2)-4*Kp2*Kp/Jx;

py_1=(alpha_y+(sqrt(beta_y))/2);
py_2=(alpha_y-(sqrt(beta_y))/2);
py_3=-cy;
py_4=0;

ty_1=atan((4/ts2)/(py_1+(4/ts2)));
ty_2=atan((4/ts2)/(py_2+(4/ts2)));
ty_3=atan((4/ts2)/(py_3+(4/ts2)));
ty_4=3*pi/4;

z_y=cy;
ty_z=atan((4/ts2)/(-z_y+(4/ts2)));

ly_1=(py_1+(4/ts2))/(cos(ty_1));
ly_2=(py_2+(4/ts2))/(cos(ty_2));
ly_3=(py_3+(4/ts2))/(cos(ty_3));
ly_4=(py_4+(4/ts2))/(cos(ty_4));
ly_z=(-z_y+(4/ts2))/(cos(ty_z));

Ky=ly_1*ly_2*ly_3*ly_4/((Kp*Kp2*g/Jx)*ly_z); 
%Ky= (py_1+4/ts2)*(py_2+4/ts2)*(py_4+4/ts2)/(Kp*Kp2*g/Jx);
Ky=abs(Ky);

%Root locus check
Pys=[py_1 py_2 py_3 py_4];

%sys2 = tf([g*Kp2*Kq/Jx z_y*g*Kp2*Kp/Jx],[1 (cy+(bp+Kp)/Jx) ((Kp*Kp2+cy*(bp+Kp))/Jx) (Kp2*Kp*cy/Jx) 0]);
%rlocus(sys2) % Y RL w zero

%% Altitude ctrler (Fast inner loop with feedforward feedback)

Tcz=tsz/25;

Kz=(m/Tcz)-cz;
Kz2= m*((4/tsz))*((-4/tsz)+((cz+Kz)/m))/Kz;

%% Control allocation (From inputs to propeller speed)

% U = C_A * W_i^2 

C_A =[     k_T           k_T          k_T                k_T          k_T         k_T;
     -k_T*L*sqrt(3)/2     0     k_T*L*sqrt(3)/2    k_T*L*sqrt(3)/2     0     -k_T*L*sqrt(3)/2;
         k_T*L/2        k_T*L       k_T*L/2           -k_T*L/2      -k_T*L       -k_T*L/2;
          +k_y          -k_y         +k_y                -k_y         +k_y          -k_y];


B=pinv(C_A);%Pseudoinverse of control allocation matrix

% B * U = W_i^2

%% Reliability of each rotors
R1=1;
R2=1;
R3=1;
R4=1;
R5=1;
R6=1;








