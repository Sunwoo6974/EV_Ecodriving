%% Car Following/SPiT. DP Forward.

% State Variables : S_host, V_host, S_front
% Input Variables : F_m, F_b (모터힘, 브레이크힘)
% 220209
% Sunwoo Kim

clc; close all;

coef = load('coefficient.mat');
VP = load('Parameter_init.mat');
MC = load('motor_spec.mat');

%% Host vehicle DP data loading and driving simulation settings
% load('aaa.mat'); % Value Table loading, J is Value table
vh_init = 10;
sh_init = 0;
%t = round(t,2); % 0 == 0 오류 발생해서 rounding 처리함

%% Vehicle Parameter Settings
m   = 1600;
phi = 0.000281250000000000; %% (1/(2m))*(rho*Cd*Af) 계산한값
g   = 9.81;
Cr  = 0.0087724;

f_brake_only = (-6000:1000:0)';
f_tire_limit = @(v) [1 1./v]*coef.c1; % Tire force min/max
p_em_dv = @(v,force) (1/v).*(v*coef.Pc(1)+v.*force*coef.Pc(2)+v.*force.^2*coef.Pc(3)+v.^3*coef.Pc(4)...
    +v.^3.*force*coef.Pc(5)+v.^5*coef.Pc(6));

%% Frontal Vehicle Data Loading
Init_gap = 100;
Precision_v = 0;
Precision_s = 0;
fv_cycle = readtable('HWFET.csv');
fv_time = table2array(fv_cycle(:,1));
vf_temp = table2array(fv_cycle(:,2))/2.237; % [Miles/Hour] -> [Meter/Second]
vf_temp(1:2) = [];
vf_temp = [vf_temp;vf_temp(end);vf_temp(end)];
vf = interp1(fv_time,vf_temp,T,'linear','extrap');
sf = td2sd(vf_temp,T_qu)+Init_gap;
%sf(1:2)=[];
%vf(1:2)=[];
vf = round(vf,Precision_v);
vf = [0 vf vf(end)];
sf = round(sf,Precision_s);

vh_save = zeros(Totalstage,1);
sh_save = zeros(Totalstage,1);

%Sh_copy = repmat(Sh,1,size(Vh,2));
%Vh_copy = repmat(Vh,size(Sh,1),1);
[Sh_copy, Vh_copy] = meshgrid(Sh,Vh);
%Sh_copy = Sh_copy';
%Vh_copy = Vh_copy';
% sh, vh, sf
vh_save(1) = vh_init;
sh_save(1) = sh_init;
%sf는 이전에 준비했음.
safe_dg = Min_dg;
for it=1:Totalstage-1
    lim_force=min(coef.const_max_force,f_tire_limit(vh_save(it)));
    f_motor_temp = (-lim_force:Force_qu:lim_force)';
    f_motor = [-lim_force*ones(size(f_brake_only,1),1);f_motor_temp]; % Motor force
    f_brake = [f_brake_only;zeros(size(f_motor_temp,1),1)]; % Brake force
    f_total = [f_brake_only-lim_force;f_motor_temp]; % Total force f_total = f_motor + f_brake
    nf = size(f_total,1);
    
    %% Running Cost
    %dyna_lin = @(s,v) 1./abs(v).*(f_total/(m) - phi*v.^2 - g*(cr*cos(slope)+sin(slope))); % 선형 미분방정식 Dynamics
    dyna_nonlin = @(t,v) f_total/m - phi*v.^2 - g*(Cr*cos(Slope(it))+sin(Slope(it))); % 비선형 미분방정식 Dynamics
    [aa,v_next_ode45_s] = ode45(dyna_nonlin,[0 T_qu],vh_save(it)*ones(size(f_total)));
    vh_next = v_next_ode45_s(end,:); % vh_next
    sh_delta = T_qu*(vh_save(it)+vh_next)/2;
    sh_next = sh_save(it)+sh_delta'; % sh_next
    

    vh_nextcopy = repmat(vh_next,size(sh_next,1),1);
    sh_nextcopy = repmat(sh_next,1,size(vh_next,2));
    vh_nextcopy = vh_nextcopy';
    sh_nextcopy = sh_nextcopy';
    
    j_power = p_em_dv(vh_save(it),f_motor)-f_brake*vh_save(it);
    
    j_dgorigin = sf(it+1)-sh_next;
    j_dg = inf*(j_dgorigin<safe_dg|j_dgorigin>100);
    j_dg(isnan(j_dg))=0;
    it
    
    sfindex = find(Sf==sf(it+1)); %%%%%%%%%%%%%%%%%%%
    
    %%% J_preint 확인할것
    transJ = permute(J{it,1}(:,:,sfindex),[2 1 3]); %%%%%%%%%%%%%%%%%%%%%%%%%%%
    %j_preint2D = interp2(Sh_copy,Vh_copy,transJ,sh_nextcopy,vh_nextcopy);
    j_preint2D = interp2(Sh_copy,Vh_copy,transJ,sh_next,vh_next,'linear');
    j_preint = diag(j_preint2D);
    %j_comb = Theta(1)*j_power+Theta(2)*j_dg+j_preint; % Backward Step #11
    j_comb = Theta(1)*j_power+j_preint; % Backward Step #11
    [JJ, index_input] = min(j_comb);

    JJ
    index_input
    F_opt_total(it) = f_total(index_input); % Optimal_Total Force
    F_opt_motor(it) = f_motor(index_input); % Optimal_Motor Force
    F_opt_brake(it) = f_brake(index_input); % Optimal_Brake Force
    
    hv_Pe_v(it) = p_em_dv(vh_save(it),F_opt_motor(it));
    Ee(it) = hv_Pe_v(it)*vh_save(it); % 모터에서 소모하는 전기적 에너지
    Em(it) = vh_save(it)*F_opt_motor(it); % 모터의 기계적 에너지
    
    % 에너지 효율 계산
    if abs(Em(it))<abs(Ee(it)) % 방전할때 (모터로써 쓸때)
        Eff_h(it) = Em(it)./Ee(it); % 기계적에너지/전기적에너지 : 전기적에너지 -> 기계적에너지 변환효율
        hv_code(it) = 1;
    else % 충전할때 (모터가 아닌 발전기로서 회생제동)
        Eff_h(it) = Ee(it)./Em(it); % 전기적에너지/기계적에너지 : 기계적에너지 -> 전기적에너지 변환효율
        hv_code(it) = -1;
    end
       
    %f2 = @(t,x) 1./abs(x).*(F_opt_total(it)/VP.m - VP.Ca*x.^2  - VP.g*(VP.Cr*cos(Slope(it))+sin(Slope(it))));
    %[aa,vh_temp] = ode45(f2,[0 T_qu],vh_save(it));
    %vh_save(it+1) = vh_temp(end); % 다음 step에서의 속도
    %sh_save(it+1) = sh_save(it) + ((vh_save(it+1)+vh_save(it))/2)*T_qu;
    vh_save(it+1) = vh_next(index_input); % 다음 step에서의 속도
    sh_save(it+1) = sh_save(it) + ((vh_save(it+1)+vh_save(it))/2)*T_qu;
    %disp('vh_real')
    %vh_next(index_input)
    %disp('sh_real')
    %vh_next(index_input)
    disp('vh')
    vh_save(it+1)
    disp('sh')
    sh_save(it+1)
    disp('sf')
    sf(it+1)
    figure(30)
    plotvalue(X,Y,Z,J{it},it,round(vh_save(it+1),1),round(sh_save(it+1),0),round(sf(it+1),0))
    man = vh_next';
    %{
    figure(18)
    surf(Sh_copy)
    colorbar;
    figure(19)
    surf(Vh_copy)
    colorbar;
    figure(20)
    plot(sh_next)
    colorbar;
    figure(21)
    plot(vh_next)
    colorbar;
    figure(99)
    surf(transJ)
    colorbar;
    %}
    
end
hv_Energy = cumtrapz(sh_save,[0; F_opt_motor'])/(3.6e6);
eff = sh_save(end)/(hv_Energy(end)*1e3);
fprintf('사용 에너지 : %d [kWh]\n', hv_Energy(end))
fprintf('주행거리 : %d [km]\n', sh_save(end)/1e3)
fprintf('전비 : %d [km/kWh]', eff)

%% Plotting
figure(1)
plot(T,vh_save,'LineWidth',2)
xlabel('Time [s]')
ylabel('Speed [m/s]')
set(gca,'Fontsize',15)
title('Host vehicle speed [m/s], for 0 to tf','Fontsize',20)
grid on

figure(2)
plot(T,sh_save,'LineWidth',2)
xlabel('Time [s]')
ylabel('Location [m]')
set(gca,'Fontsize',15)
title('Host vehicle location[m] & for 0 to tf','Fontsize',20)
grid on

figure(3)
plot(T,vf(1:Totalstage),'Color','r','LineWidth',2)
xlabel('Time [s]')
ylabel('Speed [m/s]')
set(gca,'Fontsize',15)
title('Frontal vehicle speed [m/s], for 0 to tf','Fontsize',20)
grid on

figure(4)
plot(T,sf(1:Totalstage),'Color','r','LineWidth',2)
xlabel('Time [s]')
ylabel('Location [m]')
set(gca,'Fontsize',15)
title('Frontal vehicle location[m] & for 0 to tf','Fontsize',20)
grid on

figure(5)
plot(T,vf(1:Totalstage)'-vh_save,'Color','k','LineWidth',2)
xlabel('Time [s]')
ylabel('Rel. Speed [m/s]')
set(gca,'Fontsize',15)
title('Relative speed between Host & Front & for 0 to tf','Fontsize',20)
grid on

figure(6)
plot(T,[100; sf(1:Totalstage-1)]-sh_save(1:Totalstage),'Color','k','LineWidth',2)
xlabel('Time [s]')
ylabel('Rel. Distance [m]')
set(gca,'Fontsize',15)
title('Relative distance between Host & Front & for 0 to tf','Fontsize',20)
grid on

figure(7)
plot(F_opt_total,'LineWidth',2)
xlabel('Time [s]')
ylabel('Force [N]')
set(gca,'Fontsize',15)
title('Host Vehicle Total tire force, for 0 to tf','Fontsize',20)
grid on

figure(8)
plot(F_opt_motor,'LineWidth',2)
xlabel('Time [s]')
ylabel('Force [N]')
set(gca,'Fontsize',15)
title('Host Vehicle Motor force, for 0 to tf','Fontsize',20)
grid on

figure(9)
plot(F_opt_brake,'LineWidth',2)
xlabel('Time [s]')
ylabel('Force [N]')
set(gca,'Fontsize',15)
title('Host Vehicle Brake force, for 0 to tf','Fontsize',20)
grid on

figure(10)
plot(T,hv_Energy,'LineWidth',2)
xlabel('Time [s]')
ylabel('Energy [kWh]')
set(gca,'Fontsize',15)
title('Energy Consumption, for 0 to tf','Fontsize',20)
grid on

movegui(1,[25 870])
movegui(2,[625 870])
movegui(3,[1225 870])
movegui(4,[1825 870])
movegui(5,[2425 870])
movegui(6,[25 300])
movegui(7,[625 300])
movegui(8,[1225 300])
movegui(9,[1825 300])
movegui(10,[2425 300])