function j_next = SPiT_3state_back_Val2(stage,coef,t,t_qu,sh,sh_qu,vh,vh_qu,sf,sf_qu,f_qu,slope,j_pre,totalstage,safe_dg,theta)
stage*t_qu
nsh = size(sh,1);
nvh = size(vh,2);
nsf = size(sf,1);
[X,Y,Z] = meshgrid(sh,vh,sf); % Backward Step #10.1
%{
X = permute(X, [2 1 3]);
Y = permute(Y, [2 1 3]);
Z = permute(Z, [2 1 3]);
%}

%[X1, Y1, Z1] = Jinterp(sh,vh,sf);
%j_next = -[X+Y+Z];

j_next = zeros(size(sh,1),size(vh,2),size(sf,1));
%% Vehicle Parameter / model settings
m   = 1600;
phi = 0.00028125; %% (1/(2m))*(rho*Cd*Af) 계산한값
g   = 9.81;
cr  = 0.0087724;
%wheel_r = 0.326; % Wheel radius
f_brake_only = (-6000:1000:0)';

f_tire_limit = @(v) [1 1./v]*coef.c1; % Tire force min/max
p_em_dv = @(v,force) (1/v).*(v*coef.Pc(1)+v.*force*coef.Pc(2)+v.*force.^2*coef.Pc(3)+v.^3*coef.Pc(4)...
    +v.^3.*force*coef.Pc(5)+v.^5*coef.Pc(6));

for ivh = 1:nvh
    lim_force = min(coef.const_max_force,f_tire_limit(vh(ivh)));
    f_motor_temp = (-lim_force:f_qu:lim_force)';
    f_motor = [-lim_force*ones(size(f_brake_only,1),1);f_motor_temp]; % Motor force
    f_brake = [f_brake_only;zeros(size(f_motor_temp,1),1)]; % Brake force
    f_total =[f_brake_only-lim_force;f_motor_temp]; % Total force f_total = f_motor + f_brake
    nf = size(f_total,1);
    
    %% Running Cost
    %dyna_lin = @(s,v) 1./abs(v).*(f_total/(m) - phi*v.^2 - g*(cr*cos(slope)+sin(slope))); % 선형 미분방정식 Dynamics
    dyna_nonlin = @(t,v) f_total/m - phi*v.^2 - g*(cr*cos(slope)+sin(slope)); % 비선형 미분방정식 Dynamics
    [aa,v_next_ode45_s] = ode45(dyna_nonlin,[0 t_qu],vh(ivh)*ones(size(f_total)));
    vh_next = v_next_ode45_s(end,:); % Backward Step #2
    vh_next_copy = repmat(vh_next, nsh, 1, nsf);
    j_powertemp = repmat(p_em_dv(vh(ivh),f_motor)-f_brake*vh(ivh),1,nsh,nsf); % Backward Step #3
    j_power = permute(j_powertemp,[2,1,3]); % Backward Step #4
    s_delta = ((vh(ivh)+vh_next)/2)*t_qu; % Backward Step #5
    sh_next = sh+s_delta; % Backward Step #6
    sf_copytemp = repmat(sf, 1, nsh, nf); % Backward Step #7.1
    sf_copy = permute(sf_copytemp,[2 3 1]);
    sh_next_copy = repmat(sh_next, 1, 1, nsf); % Backward Step #7.2
    j_dgorigin = sf_copy - sh_next_copy; % Backward Step #8
    %j_dg = zeros(nsh, nf, nsf); % Backward Step #9.1
    %j_dg(j_dgorigin<safe_dg)=inf; % Backward Step #9.2
    j_dg = inf*(j_dgorigin<safe_dg | j_dgorigin>100);
    j_dg(isnan(j_dg))=0;

    sh_next_copy = permute(sh_next_copy, [2 1 3]);
    vh_next_copy = permute(vh_next_copy, [2 1 3]);
    sf_copy = permute(sf_copy, [2 1 3]);

    %% Prestep Cost
    %j_preint = interp3(Y,X,Z,j_pre,sh_next_copy,vh_next_copy,sf_copy,'linear'); % Backward Step #10
    j_pretemp = permute(j_pre, [2 1 3]);
    j_preint = interp3(X,Y,Z,j_pretemp,sh_next_copy,vh_next_copy,sf_copy,'linear'); % Backward Step #10
    j_preint = permute(j_preint, [2 1 3]);

    %% Bellman Eqution
    j_comb = theta(1)*j_power+theta(2)*j_dg+j_preint; % Backward Step #11

    for ish = 1:nsh
        for isf = 1:nsf
        j_next(ish,ivh,isf) = min(j_comb(ish,:,isf));
        %{
        ish
        ivh
        isf
        j_comb(ish,:,isf)
        min(j_comb(ish,:,isf))
        %}
        end
    end
    j_next(isnan(j_next))=inf;
end
end