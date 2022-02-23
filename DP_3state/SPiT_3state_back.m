%% Car Following/SPiT. DP Backward.

% State Variables : S_host, V_host, S_front
% Input Variables : F_m, F_b (모터힘, 브레이크힘)
% 220126
% Sunwoo Kim

clear; clc; close all;
Fname = 'result20.mat';

%% Initializaion
load('Parameter_init.mat');
Coef = load('coefficient.mat');

%% Independent Variable : Time
T0 = 0;
T_qu = 1;
T_max = 50;
T = [T0:T_qu:T_max];
T = round(T,2); % 0 == 0 오류때문에 넣음

%% State Variables : Sh, Vh, Sf, Vf?
Sh_min = 0;
Sh_qu = 1;
Sh_max = 500;
Sh = [Sh_min:Sh_qu:Sh_max]'; % X,세로배열
Vh_min = 0;
Vh_qu = 1;
Vh_max = 20;
Vh = [Vh_min:Vh_qu:Vh_max]; % Y,가로배열
Sf_min = 100;   
Sf_qu = 1;
Sf_max = 850;
Sf = [Sf_min:Sf_qu:Sf_max]'; % Z,세로배열
%{
Vf_min = 0;
Vf_qu = 1;
Vf_max = 40;
Vf = [Vf_min:Vf_qu:Vf_max];
%}
%% Car Following Settings
Min_tg = 0; % Min time gap
Min_dg = 0; % Min distance gap

%% Discrete time DP
Totalstage = T_max/T_qu+1;
[X,Y,Z] = meshgrid(Vh,Sh,Sf);
%[Sh_copy, Vh_copy, Sf_copy] = Jinterp(Sh,Vh,Sf);
JTtemp = (Sf'-Sh);
JTtemp(JTtemp<0)=inf;
JT = repmat(JTtemp,1,1,size(Vh,2));
JT = permute(JT,[1 3 2]);
JTadd = zeros(size(JT));
for i=1:size(JTadd,1)
    JTadd(i,:,:) = -10*i;
end
J{Totalstage,1} = JT+JTadd;
Theta = [1e-4, 1];


Road_st = 0; % 0~8
[Slope,Height,Dist]= gen_road(Road_st,Totalstage,Sh_qu); % road slope
Force_qu = 100;
for Stage = Totalstage-1:-1:1
    Tstart = tic;
    J{Stage,1} = SPiT_3state_back_Val(Stage,Coef,T,T_qu,Sh,Sh_qu,Vh,Vh_qu,Sf,Sf_qu,Force_qu,Slope(Totalstage-Stage),J{Stage+1,1},Totalstage,Min_dg,Theta);
    TElapsed = toc(Tstart);
    fprintf('Stage : %d -> %d, %d Seconds.\n',Stage,Stage-1,TElapsed);
end

save([Fname]);
figure(1)
plotvalue(X,Y,Z,J{1},1,1,mean(Sh),100)








