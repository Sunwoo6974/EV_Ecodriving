%% IONIQ vehicle MOTOR Data Regression MOdel Estimation.
%{
   Method : Least square method.
   Code Writer : hoony.
   Model 1 : Torque limit.
   Model 2 : Input Power of Motor.
   Data :
     mc_eff_map or mc_inpwr : row(y축) w (11개 데이터), colum(x축) T (21개 데이터)
   Notation :
     w : Motor angle speed.
     v : Vehicle speed.
     T : Motor torque.
     F : Longitudinal tire force.
%}    
clear; clc; close all

%% Data Setting
if false
    run('SP_Init_data_set.m');
end
load('Parameter_init.mat') % Motor Data load

%% Model 1 : Torque limit.
% f1 = c0 + c1/v
% f2 = c0 + c1*v + c2*v^2
f1 = @(v,c) c(1) + c(2)./v;
f2 = @(v,c) c(1) + c(2).*v +c(3).*v.^2;

% Interpolation을 이용한 데이터 추가 (Method : cubic or spline or liear)
Add_map_vel = mc_map_vel(3):1:mc_map_vel(end);
Add_max_force = interp1(mc_map_vel,mc_max_force,Add_map_vel,'cubic');
nv = size(Add_map_vel,2);

% Nomal Eq : Leat square Solution.
c1 = [ones(nv,1) 1./Add_map_vel']\Add_max_force'; % Coefficients of f1
c2 = [ones(nv,1) Add_map_vel' Add_map_vel.^2']\Add_max_force'; % Coefficients of f2

rms1 = rms(Add_max_force-f1(Add_map_vel,c1));
rms2 = rms(Add_max_force-f2(Add_map_vel,c2));

% Result figure
figure(1)
plot(mc_map_vel(3:end),mc_max_force(3:end),'k-o');
hold on; grid on;
plot(Add_map_vel,Add_max_force,'g--*');
plot(Add_map_vel,f1(Add_map_vel,c1),'r--o') % 1차 regression tire force
plot(Add_map_vel,f2(Add_map_vel,c2),'b-.*') % 2차 regression tire force
xlabel('Speed of Vehicle [m/s]')
ylabel('Force of Tire [N]')
legend('Data','Data interp','f_1','f_2');
title('Motor Input Power')

% Result RMS figure
figure(2)
X = categorical({'RMS'});
X = reordercats(X,{'RMS'});
Y = [rms1,rms2];
b = bar(X,Y);
xtips1 = b(1).XEndPoints;
ytips1 = b(1).YEndPoints;
labels1 = string(b(1).YData);
text(xtips1,ytips1,labels1,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom')
xtips2 = b(2).XEndPoints;
ytips2 = b(2).YEndPoints;
labels2 = string(b(2).YData);
text(xtips2,ytips2,labels2,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom')
title('Root Mean Square')

%% Model 2 : Input Power of Motor.

vd = mc_map_vel(2:end); % except 0[m/s]
gen_force = mc_map_force(1:10); % Negative force : regeneration mode
trac_force = mc_map_force(11:end); % Positive force : traction mode
fd = [gen_force trac_force];
Add_inpwr_map = mc_inpwr_map;

%Data Extension
if true
    vd = (mc_map_vel(1)+1:(mc_map_vel(end)-mc_map_vel(1)-1)/39:mc_map_vel(end)); % 40 data
    gen_force = (mc_map_force(1):(mc_map_force(10)-mc_map_force(1))/39:mc_map_force(10)); % 40
    trac_force = (mc_map_force(11):(mc_map_force(end)-mc_map_force(11))/39:mc_map_force(end)); %40
    fd = [gen_force trac_force];
    [Xq,Yq] = meshgrid(fd,vd);
    Add_inpwr_map = interp2(mc_map_force,mc_map_vel,mc_inpwr_map,Xq,Yq);
end
nvd = size(vd,2);
ngf = size(gen_force,2);
ntf = size(trac_force,2);
num_gen=0; num_trac=0; num_total=0;

% Except over torque data
for i=1:nvd
    for j=1:ngf % regeneration data
        if gen_force(j)<-min(mc_max_force(1),abs(f2(vd(i),c2))) % over data check.
            inpwr_map_gen(i,j) = NaN;
        else
            num_gen = num_gen+1;
            data_set_gen(num_gen,1) = Add_inpwr_map(i,j);
            data_set_gen(num_gen,2) = vd(i);
            data_set_gen(num_gen,3) = fd(j);
            inpwr_map_gen(i,j) = Add_inpwr_map(i,j);
        end
    end
    for j=1+ngf:ntf+ngf % traction data
        if fd(j)>min(mc_max_force(1),abs(f2(vd(i),c2)))
            inpwr_map_trac(i,j-ngf) = NaN;
        else
            num_trac = num_trac+1;
            data_set_trac(num_trac,1) = Add_inpwr_map(i,j);
            data_set_trac(num_trac,2) = vd(i);
            data_set_trac(num_trac,3) = fd(j);
            inpwr_map_trac(i,j-ngf) = Add_inpwr_map(i,j);
        end
    end
    for j=1:ngf+ntf % total data
        if fd(j)>min(mc_max_force(1),abs(f2(vd(i),c2)))||fd(j)<-min(mc_max_force(1),abs(f2(vd(i),c2)))
            inpwr_map(i,j) = NaN;
        else
            num_total = num_total+1;
            data_set(num_total,1) = Add_inpwr_map(i,j);
            data_set(num_total,2) = vd(i);
            data_set(num_total,3) = fd(j);
            inpwr_map(i,j) = Add_inpwr_map(i,j);
        end
    end
end

% Accumulate data set 우선은 기존껄로 사용하고 교수님과 미팅후 생각해보자.
nf = 2; % Highest order
A_gen=[]; A_trac=[]; A=[];
count=0;
for i=0:nf
    for j=i:nf
        count = count+1;
        A_trac(:,count) = (data_set_trac(:,2).^(2*i+1)).*(data_set_trac(:,3).^(j-i));
        A_gen(:,count) = (data_set_gen(:,2).^(2*i+1)).*(data_set_gen(:,3).^(j-i));
        A(:,count) = (data_set(:,2).^(2*i+1)).*(data_set(:,3).^(j-i));
        basis(count) = string(['v^' num2str(2*i+1) 'F^' num2str(j-i)]);
    end
end
Pc = A\data_set(:,1);
Pe = @(c,v,f) [v v.*f v.*f.^2 v.^3 v.^3.*f v.^5]*c;
Est_inpwr_map=zeros(nvd,ngf+ntf);
for i=1:nvd
    for j=1:ngf+ntf
        if fd(j)>min(mc_max_force(1),abs(f2(vd(i),c2)))||fd(j)<-min(mc_max_force(1),abs(f2(vd(i),c2)))
            Est_inpwr_map(i,j) = nan;
        else
            Est_inpwr_map(i,j) = Pe(Pc,vd(i),fd(j));
        end
    end
end

% Result 
figure(3) % Interp2 check.
surf(mc_map_vel,mc_map_force,mc_inpwr_map')
colorbar
hold on
plot3(Yq,Xq,Add_inpwr_map,'ro')
colorbar
xlabel('Speed of Vehicle [m/s]')
ylabel('Force of Tire [N]')
zlabel('Input Power [W]')

figure(4) % Estimation result surf
surf(vd,fd,inpwr_map')
colorbar
hold on; grid on;
plot3(Yq,Xq,Est_inpwr_map','ro')
xlabel('Speed of Vehicle [m/s]')
ylabel('Force of Tire [N]')
zlabel('Input Power [W]')

figure(5) % Estimation result contour
contour(vd,fd,inpwr_map') % inpwr_map : motor의 전력 소모 회귀모
colorbar
hold on; grid on;
contour(vd,fd,Est_inpwr_map','--')
p1=plot(mc_map_vel(1:end),mc_max_force(1:end),'k');
plot(mc_map_vel(1:end),-mc_max_force(1:end),'k');
legend(p1,'Limt Force')
%plot(Add_map_vel,f2(Add_map_vel,c2),'b-.*')
xlabel('Speed of Vehicle [m/s]')
ylabel('Force of Tire [N]')
