function J_next = SPiT_CF_DP_Back_val(i1,coef,V,T,S,Vqu,Tqu,Squ,fv_V,fv_T,fv_S,Tqqu,slope,J_pre,Totalstage,theta)
%SPiT

%% Vehicle Parameter / model settings
m   = 1600;
phi = 0.000281250000000000; %% (1/(2m))*(rho*Cd*Af) 계산한값
g   = 9.81;
Cr  = 0.0087724;
tq_border = @(v) [1 1./v]*coef.c1;
tq_br = (-6000:1000:0)';
P_em_dv = @(v,force) (1/v).*(v*coef.Pc(1)+v.*force*coef.Pc(2)+v.*force.^2*coef.Pc(3)+v.^3*coef.Pc(4)...
    +v.^3.*force*coef.Pc(5)+v.^5*coef.Pc(6));
J_next = zeros(size(V,1),size(S,2));

%S_next = round(i1/10,1);

%% Backward
i1*Tqu
for iv=1:size(V,1)
    lim_tq = min(coef.const_max_force,tq_border(V(iv)));
    tq = (-lim_tq:Tqqu:lim_tq)';
    tq_wbr =[tq_br-lim_tq;tq]; % Torque with brake + motor
    tqc = [tq_br;zeros(size(tq,1),1)]; % Torque, only brake
    torque = [-lim_tq*ones(size(tq_br,1),1);tq];
    F_tire = torque/0.326;
    f_spis = @(s,v) 1./abs(v).*(tq_wbr/(m) - phi*v.^2 - g*(Cr*cos(slope)+sin(slope))); % 공간영역 정의 선형 dynamics
    %f_spit = @(s,v) ~~~ ; % 시간영역 정의 비선형 dynamics
    [aa,v_next_ode45_t] = ode45(f_spis,[0 Squ/2 Squ],V(iv)*ones(size(torque))); % ODE풀어서 next speed set save
    
    
    %% Running Cost
    V_next = v_next_ode45_t(end,:); % Backward Step #2
    %J_power = repmat(P_em_dv(V(iv),F_tire)-tqc*V(iv),1,size(S,2)); % Backward Step #3
    J_power = repmat(P_em_dv(V(iv),torque)-tqc*V(iv),1,size(S,2)); % Backward Step #3
    S_delta = ((V(iv)+V_next)/2)*Tqu; % Backward Step #4
    S_next = S+S_delta'; % Backward Step #6
    J_S = abs(S(end)-S_next);
    %J_dg = dgpreview(V_next,i1+1,S_next,fv_V,fv_T,fv_S,'backward'); % Backward Step 7 ~ 9
    J_dg = dgpreview_b(V_next,i1+1,S_next,fv_V,fv_T,fv_S,Tqu); % Backward Step 7 ~ 9
    J_runcost = theta(1)*J_dg+theta(2)*J_power+theta(3)*(J_S-i1);
    
    V_nextt = repmat(V_next,size(S_next,2),1);
    V_nextt1 = V_nextt';
    S_nextt = repmat(S_next,size(V_next,1),1);
    
    %% Pre-step cost.... for문으로 만들면 할 수 있는데
    %J_presave = J_next(V,S,J_pre',V_nextt',S_nextt)
    J_presave = interp2(S,V,J_pre,S_nextt,V_nextt1,'linear');
    
    %% Belmann Equation
    for k = 1:size(S,2)
        J_next(iv,k) = min(J_runcost(:,k)+J_presave(:,k));
    end
    
    
end
J_next(isnan(J_next))=Inf;
end
