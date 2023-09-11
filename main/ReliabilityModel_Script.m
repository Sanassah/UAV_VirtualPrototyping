% Author:       Ikedichi Nnamdi-Nwosu 
% Description:  The code in this file is supposed to be used with the
%               reliability based control allocation simulation for UAVs.
%               The main purpose of the code is to define the dynamic model
%               of the UAV being used as a testbed for the control
%               allocation algorithm.
%% Drone Dynamic Modeling

%%% Constant Parameters %%%
dt = 0.001; % sample time (s)
m = 1.45; % uav mass (kg)
mpayload = 0; % payload mass (kg)
plarm = 0; % payload moment arm (m)
k = 120; % propeller thrust to speed gain
l = 0.2; % propeller moment arm (m)
kyaw = 0.5; % propeller torque to speed constant
alpha = 1; % reliability adaptation rate
g = 9.87; % acceleration due to gravity (m/s2)
Jxx = 0.03; % moment of inertia about x-axis
Jyy = 0.03; % moment of inertia about y-axis
Jzz = 0.04; % moment of inertia about z-axis
kdrag = 5; % drag coefficient (assumed to be the same for all planes)

%%% Offset CG model %%%
cgoffset = l/2; % CG offset from geometric centre in the -z direction (m)
A = [0 0 0 0 0 0 1 0 0 0 0 0; % state matrix
     0 0 0 0 0 0 0 1 0 0 0 0;
     0 0 0 0 0 0 0 0 1 0 0 0;
     0 0 0 0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 0 0 0 0 1;
     0 0 0 g 0 0 0 0 0 0 0 0;
     0 0 0 0 -g 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 -m*g*cgoffset/Jyy 0 0 0 0 0 0 0 0;
     0 0 0 0 -m*g*cgoffset/Jxx 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0];
B = [0 0 0 0; % control matrix for controllable variables
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     1/m 0 0 0;
     0 1/Jyy 0 0;
     0 0 1/Jxx 0;
     0 0 0 1/Jzz];
C = [0 0 0 0 0 0 1 0 0 0 0 0; % output matrix
     0 0 0 0 0 0 0 1 0 0 0 0;
     0 0 0 0 0 0 0 0 1 0 0 0;
     0 0 0 0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 0 0 0 0 1];
D = zeros(6,8); % output control matrix

%%% Drag Addition %%%
A = A + [0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 -kdrag/m 0 0 0 0 0;
     0 0 0 0 0 0 0 -kdrag/m 0 0 0 0;
     0 0 0 0 0 0 0 0 -kdrag/m 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0];

%%% Attached Payload Addition %%%
% mpayload = 0.5; % payload mass (kg)
% plarm = l/2; % payload moment arm (m)
 
%% Drone Controller Design
% Controllability check
Cx = ctrb(A,B); % Controllability Matrix:
check1 = rank(Cx);
if check1 == size(Cx,1)
    disp('System is controllable')
elseif check1 < size(Cx,1)
    disp('System is uncontrollable')
end

% Desired response parameters
ts_c = 0.5; % settling time [s]
os_c = 1.05; % overshoot
tau_c = ts_c/4; % time constant [s]

% Characteristic polynomial variables
zeta_c = log(1/(os_c-1))/sqrt((log(1/(os_c-1)))^2+pi^2); % damping ratio
wn_c = 1/(tau_c*zeta_c); % natural frequency (rad/s)

% Poles for desired response
p_c = roots([1 2*zeta_c*wn_c wn_c^2]);
Pdes_c = [p_c' p_c' p_c'*1.001 p_c'*1.001 p_c'*1.002 p_c'*1.002];

% Controller gain matrix calculation
%KJosh = place(A,B,Pdes_c);

% Tracking gain calculation
%Kr = KJosh(:,1:6);
% Kr = ([K eye(size(K,1),(size([A';B'],1)-size(K,2)))]/statematrix)*[zeros(size([A' C'],1),6);eye(size(C',2),6)];

%% Drone Kinematics Modeling
thrust2inp = [1 1 1 1 1 1 1 1; % control effectiveness matrix
              l -l 0 0 l -l 0 0;
              0 0 l -l 0 0 l -l;
              kyaw kyaw -kyaw -kyaw kyaw kyaw -kyaw -kyaw];
inp2thrust = pinv(thrust2inp); % control effectiveness matrix inverse
Bcnt = B*thrust2inp; % control matrix for operable control (motors)

%% Drone Propeller Modeling Parameters
%%% Atmospheric Conditions %%%
% Constant Parameters
p0 = 101325; % Standard Pressure (Pa)
T0 = 15; % Standard Temperature (C)
rho0 = 1.225; % Standard Density (kg/m^3)
lapse = 6.5*(1/1000); % Temperature Lapse Rate (C/m)
gamma = 1.4; % Thermal Capacity Ratio for air
Hp = 233; % drone operation altitude (m)
DISA = 0; % temperature deviation from ISA (C)

% Calculate pressuere ratio based on atmosphere location
if Hp < 36089
    delta = (1-6.87535*10^(-6)*Hp)^5.2559;
else
    delta = exp((36089-Hp)/20806)/4.477;
end

% Calculate static pressure
p = delta*p0;

% Calculate standard temperature
Tisa = T0 - lapse*Hp;
if Hp >= 36089 % Standard Temperaute is constant above 36089ft
    Tisa = -56.50;
end

% Calculate temperature from deviation from ISA
T = Tisa + DISA;

% Calculate static temperature ratio
theta = (T+273.15)/(T0+273.15);

% Calculate density ratio
sigma = delta/theta;

% Calculate density
rho = sigma*rho0;

%%% Propeller sizing parameters %%%
Bp = 2; % propeller number
Dp = 10; % propeller diameter (in)
hp = 4.5; % propeller pitch (in)
a = 5;
epsilon = 0.85;
lamda = 0.75;
eta = 0.5;
e = 0.83;
Cfd = 0.015;
alp0 = 0;
K0 = 6.11;
Cd = Cfd+(pi*a*K0^2/e)*((epsilon*atan(hp/(pi*Dp))-alp0)^2/(pi*a+K0)^2);
CT = 0.25*pi^3*lamda*eta^2*Bp*K0*((epsilon*atan(hp/(pi*Dp))-alp0)/(pi*a+K0));
CM = pi^2*Cd*eta^2*lamda*Bp^2/(8*a);

%% Drone Motor Modeling Parameters
nr = 8; % number of propellers/motors/actuators
W = m*g; % drone weight (N)
Kv0 = 890; % KV rating [rpm/v]
Immax = 19; % maximum motor current [v]
Im0 = 0.5; % nominal no-load current [A]
Um0 = 10; % nominal no-load voltage [A]
Rm = 0.101; % motor resistance [ohm]
Gm = 1;

%% ESC & Battery Modeling Parameters
Cb = 5000;
Rb = 0.01; % battery resistance [ohm]
Ub = 12;
Kb = 45;
Gb = 10;
Iemax = 30; % maximum ESC current [v]
Re = 0.008; % ESC resistance [ohm]
Ge = 10;
Iother = 1;

%% Reliability Model Parameters
pconst = 2; %arbitrarily because outside scope of project
bconst = 1.2; %arbitrarily because outside scope of project

%% Reliability Model Parameters 

%Electrolyte vaporisation for capacitor
nEVCAP = 1; % value of 0 for axial capacitor and 1 for radial capacitor
LnomEVCAP = 3000*3600; % nominal capacitor lifetime for EVCAP (hours)
VnomEVCAP = 35; % nominal voltage (volt)
VmaxEVCAP = 35; % same as nominal voltage (volts)
EaEVCAP = 0.94; % activation energy (electronvolts)
TrefEVCAP = 105; % reference temperature (celcius)
RrefEVCAP = 0.9; % reference reliability (assumption)
bEVCAP = 1.1; % weibull shape factor (assumption)

%%Electromigration
% AeEMSC = 0.5;
% NEMSC = 0.5;
% EaEMSC = 0.5;
% TrefEMSC = 0.5;
% LnomEMSC = 0.5;
% JrefEMSC = 0.5;
% RrefEMSC = 0.5;
% bEMSC = 0.5;

%Semiconductor TDDB
EaTDDBSC = 0.7; % activation energy (electronvolts)
TrefTDDBSC = 150; % reference temperature (celcius)
RrefTDDBSC = 0.37; % reference reliability (assumption)
bTDDBSC = 1.1; % weibull shape factor (assumption)
LnomTDDBSC = 2777*3600; % nominal capacitor lifetime for Semiconductor (hours)
EalphaTDDBSC = 0.7; % electric field strength (volts/meter)
alpha1TDDBSC = 0.7; % value for silicon carbide molecule but we are using silicon dioxyde

% Winding 
EaWDG = 0.5; % activation energy (electronvolts)
TrefWDG = 40; %reference temperature
RrefWDG = 0.98; %reference reliability
bWDG = 1.1; %need to confirm with dr. liscouet
LnomWDG = 20000*3600; % nominal winding lifetime(hours)

R_th_esc = 5.14; %esc thermoresistance (degrees celcius per watt)
T_amb = 20; % ambient temperature (degrees celcius)

C_th_esc = 1.67; % thermal capacitance of esc (jules per kelvin)


R_th_mot = 8.59; % thermal motor resistance (degrees celcius per watt)
T_amb_mot = 20; % ambient temperature

C_mot = 17.46; % thermal capacitor resistance (jules per kelvin)

k_b = 1.38*e-23; % boltzmann constant

bBNG = 1.2; % weibull shape factor for the bearing
RrefBNG = 0.9; % reference reliability bbearing
t_D_SCTDDB = 30; %thickness of the gate oxide


%% Initial conditions for reliability 

%Battery Initial Condition
In_SOC_Bat1 = 1;
In_SOC_Bat2 = 1;

%Reliability Observer (CAPEV)

CAPEV_Motor1 = 0.5;
CAPEV_Motor2 = 0.5;
CAPEV_Motor3 = 0.5;
CAPEV_Motor4 = 0.5;
CAPEV_Motor5 = 0.5;
CAPEV_Motor6 = 0.5;

%Reliability Observer (SCTDDB)

SCTDDB_Motor1 = 0.5;
SCTDDB_Motor2 = 0.5;
SCTDDB_Motor3 = 0.5;
SCTDDB_Motor4 = 0.5;
SCTDDB_Motor5 = 0.5;
SCTDDB_Motor6 = 0.5;

%Reliability Observer (SCEM)

SCEM_Motor1 = 1;
SCEM_Motor2 = 1;
SCEM_Motor3 = 1;
SCEM_Motor4 = 1;
SCEM_Motor5 = 1;
SCEM_Motor6 = 1;

%Reliability Observer (WDG)

WDG_Motor1 = 1;
WDG_Motor2 = 1;
WDG_Motor3 = 1;
WDG_Motor4 = 1;
WDG_Motor5 = 1;
WDG_Motor6 = 1;

%Reliability Observer (BNG)

BNG_Motor1 = 1;
BNG_Motor2 = 1;
BNG_Motor3 = 1;
BNG_Motor4 = 1;
BNG_Motor5 = 1;
BNG_Motor6 = 1;