%% Car Following
% 220110
% Sunwoo Kim

clear; clc; close all;

%% Initializaion
load('Parameter_init.mat');
coef = load('coefficient.mat');

%% Parameter Settings


%% Host vehicle DP settings
Vmin = 0;
Vqu = 0.1;
Vmax = 40;
T0 = 0;
Tqu = 0.1;
Tf = 30;
S0 = 0;
Squ = 0.1;
Sf = 1000;
Squ = 1;
Sf = 10;
V = [Vmin:Vqu:Vmax]';
T = [T0:Tqu:Tf]';
T = round(T,1); % 0 == 0 오류 발생해서 rounding 처리함
S = [S0:Squ:Sf]';
%nv = size(V,1);
%nt = size(T,1);

%% Host vehicle DP data loading and driving simulation settings
init_gap = 20;
fv_cycle = readtable('HWFET.csv');
fv_timetemp = table2array(fv_cycle(:,1));
fv_vtemp1 = table2array(fv_cycle(:,2))/2.237; % [km/h] -> [m/s]
fv_T = [0:0.1:fv_timetemp(end)]';
fv_Vtemp = interp1(fv_timetemp,fv_vtemp1,fv_T,'linear');
fv_V = td2sd(fv_Vtemp)+init_gap; % 거리
fv_S = cumsum(fv_V);


%% Discrete time DP
Totalstage = Sf/Squ %+1; % spis
%Totalstage = Tf/Tqu+1; % spit
J{Totalstage,1} = 0; %repmat((1-lam)*T,1,nv); % Final stage cost (3D)

%% Road type 결정
road_st = 0; % 0~8
[slope,height,dist]= gen_road(road_st,Totalstage,Squ); % road slope

%% Backward propagation - 1부터 TotalStage까지. Final 제외
Tqqu = 100;
for i1=Totalstage-1:-1:1
    tstart = tic;
    
    J{i1,1} = CF_DP_Back_Val(i1,coef,V,T,S,Vqu,Tqu,Squ,fv_V,fv_T,fv_S,Tqqu,slope(Totalstage-i1),J{i1+1,1},Totalstage);
    
    tElapsed = toc(tstart);
    fprintf('Stage : %d -> %d, %d Seconds.\n',i1,i1-1,tElapsed);
end
save([fname]);

surf(J{1});
colorbar;
