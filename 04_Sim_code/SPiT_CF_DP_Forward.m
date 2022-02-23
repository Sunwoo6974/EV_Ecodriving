%% Car Following/SPiT. DP Forward.
% 220119
% Sunwoo Kim

clc; close all;

coef = load('coefficient.mat');
VP = load('Parameter_init.mat');
MC = load('motor_spec.mat');

%% Host Vehicle DP Data loading, and Driving Simulation Settings.
%load('results.mat');
hv_init_v = 0.1;
hv_init_s = 0;

%% Vehicle Parameter Settings
m   = 1600;
phi = 0.000281250000000000; %% (1/(2m))*(rho*Cd*Af) 계산한값
g   = 9.81;
Cr  = 0.0087724;
tq_border = @(v) [1 1./v]*coef.c1;
lim_force = @(v) [1 1./v]*coef.c1;
hv_tq_br = (-6000:1000:0)';
hv_lim_tq = 0;
P_em_dv = @(v,f) (1/v).*(v*coef.Pc(1)+v.*f*coef.Pc(2)+v.*f.^2*coef.Pc(3)+v.^3*coef.Pc(4)...
    +v.^3.*f*coef.Pc(5)+v.^5*coef.Pc(6));

%% Frontal vehicle driving profile loading & Preprocessing and re-quantization, Result에서 가져오기 때문에 안해도 됨
%init_gap = 20;
%fv_cycle = readtable('HWFET.csv');
%fv_timetemp = table2array(fv_cycle(:,1));
%fv_vtemp1 = table2array(fv_cycle(:,2))/2.237; % [mile/h] -> [m/s]
%fv_T = [0:0.1:fv_timetemp(end)]';
%fv_Vtemp = interp1(fv_timetemp,fv_vtemp1,fv_T,'linear');
%fv_V = 0;
%fv_S = td2sd(fv_Vtemp)*0.1+init_gap;
%fv_S = cumsum(fv_V);

%% Discrete time DP Forward

hv_vsave = zeros(Totalstage,1);
hv_ssave = zeros(Totalstage,1);
hv_Pe = zeros(Totalstage,1);
hv_Pe_v = zeros(Totalstage,1);
hv_Pm = zeros(Totalstage,1);

hv_vsave(1) = hv_init_v;
hv_ssave(1) = hv_init_s;

%% Road type, Result에서 가져오기 때문에 안해도 됨
%road_st = 0; % 0~8
%[slope,height,dist]= gen_road(road_st,Totalstage+2,SD); % road slope

for it = 1:Totalstage-1
    hv_lim_tq = min(coef.const_max_force,lim_force(hv_vsave(it)));
    hv_tq = (-hv_lim_tq:Tqqu:hv_lim_tq)';
    hv_tq_wbr =[hv_tq_br-hv_lim_tq;hv_tq]; % Torque with brake + motor
    hv_tqc = [hv_tq_br;zeros(size(hv_tq,1),1)];
    hv_torque = [-hv_lim_tq*ones(size(hv_tq_br,1),1);hv_tq];
    hv_F_tire = hv_torque/0.326;

    hv_F_spis = @(s,v) 1./abs(v).*(hv_tq_wbr/(m) - phi*v.^2 - g*(Cr*cos(slope(it))+sin(slope(it)))); % 공간영역 정의 선형 dynamics
    %hv_F_spit = @(t,v) 1./abs(v).*(hv_tq_wbr/(m) - phi*v.^2 - g*(Cr*cos(slope(it))+sin(slope(it)))); % 시간영역 정의 비선형 dynamics
    [aa,v_next_ode45_t] = ode45(hv_F_spis,[0 Squ/2 Squ],hv_vsave(it)*ones(size(hv_torque))); % ODE풀어서 next speed set save

    %% Running Cost
    V_next = v_next_ode45_t(end,:)';
    J_power = P_em_dv(hv_vsave(it),hv_torque)-hv_tqc*hv_vsave(it); % Backward Step #3
    %J_power = P_em_dv(hv_vsave(it),hv_F_tire)-hv_tqc*hv_vsave(it); % Backward Step #3
    S_delta = ((hv_vsave(it)+V_next)/2)*Tqu;
    S_next = hv_ssave(it)+S_delta';
    J_S = abs(S(end)-S_next);
    %J_dg = dgpreview(V_next,it+1,S_next,fv_V,fv_T,fv_S,'forward');
    J_dg = dgpreview_f(V_next,it+1,S_next,fv_V,fv_T,fv_S,Tqu);
    J_runcost = theta(1)*J_dg+theta(2)*J_power'+theta(3)*(J_S-it);
    
    %Interpolation 하면 안되고... J{it+1}에서 (S_next, V_next)에 해당하는 딱 그 값을 찾아야함
    J_pre = diag(interp2(S,V,J{it+1},S_next,V_next,'linear'))'; % next step value table impor

    [JJ, index_input] = min(J_runcost+J_pre);

    hv_opt_total_input(it) = hv_tq_wbr(index_input)/0.326; % Optimal_Total Force
    hv_opt_Fm_input(it) = hv_torque(index_input)/0.326; % Optimal_Motor Force
    hv_opt_Fb_input(it) = hv_tqc(index_input)/0.326; % Optimal_Brake Force
    
    hv_Pe_v(it) = P_em_dv(hv_vsave(it),hv_opt_Fm_input(it));
    hv_Pe(it) = hv_Pe_v(it)*hv_vsave(it); % 모터에서 소모하는 전기적 에너지
    hv_Pm(it) = hv_vsave(it)*hv_opt_Fm_input(it); % 모터의 기계적 에너지
    
    % 에너지 효율 계산
    if abs(hv_Pm(it))<abs(hv_Pe(it)) % 방전할때 (모터로써 쓸때)
        hv_eff(it) = hv_Pm(it)./hv_Pe(it); % 기계적에너지/전기적에너지 : 전기적에너지 -> 기계적에너지 변환효율
        hv_code(it) = 1;
    else % 충전할때 (모터가 아닌 발전기로서 회생제동)
        hv_eff(it) = hv_Pe(it)./hv_Pm(it); % 전기적에너지/기계적에너지 : 기계적에너지 -> 전기적에너지 변환효율
        hv_code(it) = -1;
    end
    
    % f2가 hv_opt_total_input(파워)로 들어가는데 f와 비교해볼것. f에서는 파워가 아닌 토크로 들어감. 확인해볼것.
    f2 = @(t,x) 1./abs(x).*(hv_opt_total_input(it)/VP.m - VP.Ca*x.^2  - VP.g*(VP.Cr*cos(slope(it))+sin(slope(it))));
    [aa,hv_vt] = ode45(f2,[0 Squ/2 Squ],hv_vsave(it));
    hv_vsave(it+1) = hv_vt(end); % 다음 step에서의 속도
    hv_ssave(it+1) = hv_ssave(it) + ((hv_vsave(it)+hv_vsave(it+1))/2)*Tqu;
end
%{
hv_Ee = cumtrapz(T,hv_Pe)/(3.6e6)
figure(1)
plot(T,hv_Pe/1e3)
figure(2)
plot(T,hv_Ee)
%}

hv_Energy = cumtrapz(hv_ssave,[0; hv_opt_Fm_input'])/(3.6e6);
figure(10)
plot(T,hv_Energy)



%% Plotting
figure(1)
plot(T,hv_vsave,'LineWidth',2)
xlabel('Time [s]')
ylabel('Speed [m/s]')
set(gca,'Fontsize',15)
title('Host vehicle speed [m/s], for 0 to tf','Fontsize',20)
grid on

figure(2)
plot(T,hv_ssave,'LineWidth',2)
xlabel('Time [s]')
ylabel('Location [m]')
set(gca,'Fontsize',15)
title('Host vehicle location[m] & for 0 to tf','Fontsize',20)
grid on

figure(3)
plot(T,fv_Vtemp(2:Totalstage+1),'LineWidth',2)
xlabel('Time [s]')
ylabel('Speed [m]')
set(gca,'Fontsize',15)
title('Frontal vehicle speed [m/s], for 0 to tf','Fontsize',20)
grid on

figure(4)
plot(T,fv_S(2:Totalstage+1),'LineWidth',2)
xlabel('Time [s]')
ylabel('Location [m]')
set(gca,'Fontsize',15)
title('Frontal vehicle location[m] & for 0 to tf','Fontsize',20)
grid on

figure(5)
plot(T,fv_Vtemp(2:Totalstage+1)-hv_vsave,'LineWidth',2)
xlabel('Time [s]')
ylabel('Rel. Speed [m/s]')
set(gca,'Fontsize',15)
title('Relative speed between Host & Front & for 0 to tf','Fontsize',20)
grid on

figure(6)
plot(T,fv_S(2:Totalstage+1)-hv_ssave,'LineWidth',2)
xlabel('Time [s]')
ylabel('Rel. Distance [m]')
set(gca,'Fontsize',15)
title('Relative distance between Host & Front & for 0 to tf','Fontsize',20)
grid on

%{
figure(7)
plot(T,hv_Pe_v,'LineWidth',2)
xlabel('Time [s]')
ylabel('Motor overall Power')
set(gca,'Fontsize',15)
title('Host Vehicle Overall Power for 0 to tf','Fontsize',20)
grid on

%dist_feasiblemin = max(0,(fv_S-40));

figure(8)
fill(fv_S(1:Totalstage),dist_feasiblemin(1:Totalstage),'g');
xlabel('Time [s]')
ylabel('Distance [m]')
set(gca,'Fontsize',15)
title('Host Vehicle Feasible location for each time 0 ~ tf','Fontsize',20)
grid on

figure(9)
plot(T,hv_Ee,'LineWidth',2)
xlabel('Time [s]')
ylabel('Energy [kWh]')
set(gca,'Fontsize',15)
title('Host Vehicle Energy Consumption for each time 0 ~ tf','Fontsize',20)
grid on

cum_elec_eff = (S(end)*1e-3)./hv_Ee(end);

fprintf('Efficiency : %d [km/kWh]\n', cum_elec_eff)
%}