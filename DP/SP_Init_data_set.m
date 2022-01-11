%% IONIQ vehicle Data Setting, But Moter data is other vehicle data
%{
   Code Writer : hoony.
   Notation :
     mc_map_spd : Motor angle speed.
     mc_map_vel : Vehicle speed.
     mc_map_trq : Motor torque.
     mc_map_force : Longitudinal tire force .
%}    

load('Data_mat/motor_spec.mat') % Motor Data load

% Vehecle parametor
n_mot = 1 ; % number of motor
r_wh  = 652*10^-3/2; % wheel radius [m]
g_r   = 7.4; % Gear ratio
m = 1600; % velhicle total weight[kg]
Iw = 1; % mass moment of inertia[kg*m^2]
Ie = 0.25; % mass moment of inertia[kg*m^2]
g =9.81; % gravitational acceleration [m/s^2]
Cr = 0.0087724; % Friction coefficient between road and tire
CdAf = 0.75; % Cd : Aerodynamic drag coefficient, Af : Front area of the vehicle
rhoa = 1.2; % density of air [kg/m^3]
Ca = 0.5*CdAf*rhoa/m;

% Rotaion speed and Torque to Tire fore and Vehicle Speed.
mc_map_vel   = mc_map_spd*(r_wh/g_r); % [m/s]
mc_map_force = mc_map_trq*(n_mot*g_r/r_wh); % [N]
mc_max_force = mc_max_trq*(n_mot*g_r/r_wh);  % [N]

path = pwd;
save([path '/Parameter_init']);
