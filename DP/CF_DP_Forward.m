%% CF_DP_Forward.m

clc; close all

coef = load('coefficient.mat');
VP = load('Parameter_init.mat'); % 9900Km
MC = load('motor_spec.mat'); % 9900Km

%% Host vehicle DP data loading and driving simulation settings
% load('aaa.mat'); % Value Table loading, J is Value table
hv_init_v = 10;
hv_init_t = 0;

t = round(t,1); % 0 == 0 오류 발생해서 rounding 처리함

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

%% Road type 결정
road_st = 0; % 0~8
[slope,height,dist]= gen_road(road_st,Totalstage+2,SD); % road slope

%% Frontal Vehicle Data loading and preprocessing : WLTP Class 1 Vehicle
fv_cycle = readtable('fv_200.csv'); % 99
%fv_cycle = readtable('wltp_class1.csv'); % 99
fv_timetemp = table2array(fv_cycle(:,1));
fv_v = table2array(fv_cycle(:,2))/3.6; % [km/h] to [m/s]
fv_s = td2sd(fv_v); % frontal vehicle location / spatial domain
fv_t = (0:0.1:100)';
fv_t = round(fv_t,1);
hv_dg = 30;
fv_inter_s = hv_dg + interp1(fv_timetemp,fv_s,fv_t,'linear','extrap');


hv_vsave = zeros(Totalstage+1,1);
hv_tsave = zeros(Totalstage+1,1);
hv_Pe = zeros(Totalstage,1);
hv_Pe_v = zeros(Totalstage,1);
hv_Pm = zeros(Totalstage,1);

hv_eff = zeros(Totalstage,1);
hv_opt_total_input = zeros(Totalstage,1);
hv_opt_Fm_input = zeros(Totalstage,1);
hv_opt_Fb_input = zeros(Totalstage,1);

hv_vsave(1) = hv_init_v;
hv_tsave(1) = hv_init_t;
hv_dgsave(1) = fv_inter_s(1);

for iv=1:Totalstage
    
    hv_lim_tq = min(coef.const_max_force,lim_force(hv_vsave(iv)));
    hv_tq = (-hv_lim_tq:tqqu:hv_lim_tq)';
    hv_tq_wbr =[hv_tq_br-hv_lim_tq;hv_tq]; % Torque with brake + motor
    hv_tqc = [hv_tq_br;zeros(size(hv_tq,1),1)];
    hv_torque = [-hv_lim_tq*ones(size(hv_tq_br,1),1);hv_tq];
    f = @(t,spd) 1./abs(spd).*(hv_tq_wbr/(m) - phi*spd.^2 - g*(Cr*cos(slope(iv))+sin(slope(iv))));
 
    [aa,v_next_ode45_t] = ode45(f,[0 SD/2 SD],hv_vsave(iv)*ones(size(hv_torque))); % ODE풀어서 next speed set save
    v_next = v_next_ode45_t(end,:)';
    
    delta_t = 2*SD./(hv_vsave(iv)+v_next);
    t_next = hv_tsave(iv)+delta_t';
    
    vrepeat = repmat(v_next,1,nt);
    
    dg_st1 = (v_next.*(v_next>=16.6666))*3.6;
    dg_st2 = (v_next-(15/3.6)).*(v_next<16.6666)*3.6;
    dg_st = dg_st1+dg_st2;
    
    %{
    disp("size of t_next', v_next");
    size(t_next')
    size(v_next)
    size(hv_vsave)
    size(delta_t')
    %}
    
    J_t = interp2(t,v,J{iv+1},t_next',v_next); % 131 x 1
    J_power = P_em_dv(hv_vsave(iv),hv_torque)-hv_tqc*hv_vsave(iv); % 131 x 1
    J_dgtemp = J_t*0;
    
    for it = 1:size(t_next',1)
        temp = fv_s(find(t_next(it) == fv_t))-iv;
        if isempty(temp) == 1
            J_dgtemp(it) = 0;
        else
            J_dgtemp(it) = temp;
            disp("nnnnnnnnnn");
        end
    end
    
    J_dg = abs(dg_st - J_dgtemp);
    
    [JJJ,index_input] = min(lam*SD*theta(1)*J_power+theta(2)*J_t+theta(3)*J_dg);
    
    hv_opt_total_input(iv) = hv_tq_wbr(index_input); % Total Power
    hv_opt_Fm_input(iv) = hv_torque(index_input); % Motor Power
    hv_opt_Fb_input(iv) = hv_tqc(index_input); % Brake Power
    
    hv_Pe_v(iv) = P_em_dv(hv_vsave(iv),hv_opt_Fm_input(iv));
    hv_Pe(iv) = hv_Pe_v(iv)*hv_vsave(iv); % 모터에서 소모하는 전기적 에너지
    %hv_Pm(iv) = v(iv)*hv_opt_Fm_input(iv); % 모터의 기계적 에너지
    hv_Pm(iv) = hv_vsave(iv)*hv_opt_Fm_input(iv); % 모터의 기계적 에너지
    
    % 에너지 효율 계산
    if abs(hv_Pm(iv))<abs(hv_Pe(iv)) % 방전할때 (모터로써 쓸때)
        hv_eff(iv) = hv_Pm(iv)./hv_Pe(iv); % 기계적에너지/전기적에너지 : 전기적에너지 -> 기계적에너지 변환효율
    else % 충전할때 (모터가 아닌 발전기로서 회생제동)
        hv_eff(iv) = hv_Pe(iv)./hv_Pm(iv); % 전기적에너지/기계적에너지 : 기계적에너지 -> 전기적에너지 변환효율
    end
    
    
    f2 = @(t,x) 1./abs(x).*(hv_opt_total_input(iv)/VP.m - VP.Ca*x.^2  - VP.g*(VP.Cr*cos(slope(iv))+sin(slope(iv))));
    [aa,hv_vt] = ode45(f2,[0 SD/2 SD],hv_vsave(iv));
    hv_vsave(iv+1) = hv_vt(end); % 다음 step에서의 속도
    hv_tsave(iv+1) = hv_tsave(iv) + 2*SD/(hv_vsave(iv+1)+hv_vsave(iv));   

end

disttemp = 0:1:size(hv_Pe_v,1)-1;
hv_Ee = cumtrapz(disttemp,hv_Pe_v);
hv_Em = cumtrapz(disttemp,hv_opt_Fm_input);
hv_Em(end)
hv_Ee(end)




figure(1)
plot(0:1:Totalstage,hv_vsave,'LineWidth',2)
xlabel('Travel distance [m]')
ylabel('Speed [m/s]')
set(gca,'Fontsize',15)
title('Host vehicle speed [m/s], for 0 to 200[m]','Fontsize',20)
grid on

figure(2)
plot(hv_Ee)

figure(3)
plot(hv_tsave,0:1:Totalstage,'LineWidth',2)
ylabel('Travel distance [m]')
xlabel('Travel time [s]')
set(gca,'Fontsize',15)
title('Host vehicle time[s] & location[m]','Fontsize',20)
grid on


figure(4)
plot(t,fv_inter_s(1:size(t,1)),'LineWidth',2)
ylabel('Travel distance [m]')
xlabel('Travel time [s]')
set(gca,'Fontsize',15)
title('frontal vehicle time[s] & location[m]','Fontsize',20)
grid on


hv_time = zeros(size(hv_vsave));

for iii = 2:1:size(hv_vsave,1)
    hv_time(iii) = 2*SD/(hv_vsave(iii-1)+hv_vsave(iii));
end

NRG = zeros(Totalstage,1);

for kk1 = 1:Totalstage
    NRG1(kk1) = 0.001/((hv_Pe_v(kk1)/1000)*(hv_time(kk1+1)/3600));
end

for kk2 = 1:Totalstage
    NRG2(kk2) = hv_Pe_v(kk2) * (hv_time(kk2+1)/3600) / 1000;
end

%ef1 = mean(NRG1);
ef2 = 0.2/sum(NRG2);

NRG_sum = cumsum(NRG2);






hv_timeall = sum(hv_time)
hv_avgspe = (Totalstage-1)/hv_timeall
    


%hv_Ee = cumtrapz(dist(1:end-1),hv_Pe_v)
%hv_Em = cumtrapz(dist(1:end-1),hv_opt_Fm_input)