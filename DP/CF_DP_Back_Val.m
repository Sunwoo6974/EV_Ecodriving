function J_next = CF_DP_Back_val(i1,coef,V,T,S,Vqu,Tqu,Squ,fv_V,fv_T,fv_S,slope,J_pre);

%% Vehicle Parameter settings
m   = 1600;
phi = 0.000281250000000000; %% (1/(2m))*(rho*Cd*Af) 계산한값
g   = 9.81;
Cr  = 0.0087724;
tq_border = @(v) [1 1./v]*coef.c1;
tq_br = (-6000:1000:0)';
P_em_dv = @(v,f) (1/v).*(v*coef.Pc(1)+v.*f*coef.Pc(2)+v.*f.^2*coef.Pc(3)+v.^3*coef.Pc(4)...
    +v.^3.*f*coef.Pc(5)+v.^5*coef.Pc(6));
J_next = zeros(size(V,1),size(T,1));

%% Backward
for iv=1:size(V,1)
    lim_tq = min(coef.const_max_force,tq_border(V(iv)));
    tq = (-lim_tq:Tqu:lim_tq)';
    tq_wbr =[tq_br-lim_tq;tq]; % Torque with brake + motor
    tqc = [tq_br;zeros(size(tq,1),1)];
    torque = [-lim_tq*ones(size(tq_br,1),1);tq];
    f = @(t,v) 1./abs(v).*(tq_wbr/(m) - phi*v.^2 - g*(Cr*cos(slope)+sin(slope)));
    [aa,v_next_ode45_t] = ode45(f,[0 Squ/2 Squ],V(iv)*ones(size(torque))); % ODE풀어서 next speed set save
    V_next = v_next_ode45_t(end,:)';
    delta_T = 2*Squ./(V(iv)+V_next);
    T_next = T+delta_T';
    man = dgpreview(V_next,T_next,fv_V,fv_T,fv_S)
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
   
    
    
    
end
J_next(isnan(J_next))=Inf;
end
