%% Car Following/SPiT. DP Backward.
% 220110
% Sunwoo Kim

clear; clc; close all;
fname = 'spit16.mat';

%% Initializaion
load('Parameter_init.mat');
coef = load('coefficient.mat');

%% Host vehicle DP settings
Vmin = 0;
Vqu = 0.1;
Vmax = 40;
T0 = 0;
Tqu = 0.5;
Tf = 45;
S0 = 0;
Squ = 0.5;
Sf = 500;
V = [Vmin:Vqu:Vmax]';
T = [T0:Tqu:Tf]';
T = round(T,1); % 0 == 0 오류 발생해서 rounding 처리함
S = [S0:Squ:Sf];
%nv = size(V,1);
%nt = size(T,1);

%% Frontal vehicle driving profile loading & Preprocessing and re-quantization
init_gap = 50;
fv_cycle = readtable('HWFET.csv');
fv_timetemp = table2array(fv_cycle(:,1));
fv_vtemp1 = table2array(fv_cycle(:,2))/2.237; % [mile/h] -> [m/s]
fv_T = [0:Tqu:fv_timetemp(end)]';
fv_Vtemp = interp1(fv_timetemp,fv_vtemp1,fv_T,'linear');
fv_V = 0;
fv_S = td2sd(fv_Vtemp,Tqu)+init_gap;
%fv_S = cumsum(fv_V);

%% Discrete time DP
%Totalstage = Sf/Squ %+1; % spis
Totalstage = Tf/Tqu+1; % spit
%J{Totalstage,1} = -(V+S.^3);%-10*(S+V).^2; %repmat((1-lam)*T,1,nv); % Final stage cost (3D)
J{Totalstage,1} = -(V.^5+S.^5);
%J{Totalstage,1}(:,end) = -1e-10;
theta = [1, 1e-7, 1]; % 1000이상, 1e-5 ~ 1e-6, 0.1~1 // J_dg, J_power, J_S

%% Road type 결정
road_st = 0; % 0~8
[slope,height,dist]= gen_road(road_st,Totalstage,Squ); % road slope

%% Backward propagation - 1부터 TotalStage까지. Final 제외
Tqqu = 100;
for i1=Totalstage-1:-1:1
    tstart = tic;
    
    J{i1,1} = SPiT_CF_DP_Back_Val(i1,coef,V,T,S,Vqu,Tqu,Squ,fv_V,fv_T,fv_S,Tqqu,slope(Totalstage-i1),J{i1+1,1},Totalstage,theta);
    
    tElapsed = toc(tstart);
    fprintf('Stage : %d -> %d, %d Seconds.\n',i1,i1-1,tElapsed);
end
save([fname]);

figure(1)
surf(S,V,J{1});
colorbar;
xlabel('Location')
ylabel('Speed')

%% 이제 Time step T에서 value table이 바뀌어야 함. 지금은 어느 step이든 value table이 똑같음. 어떤 방법으로 해결?
