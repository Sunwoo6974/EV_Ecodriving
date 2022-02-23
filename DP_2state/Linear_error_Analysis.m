%% The Spatial Domain : Longitudinal Dynanics Linearizaion error Analysis.
%{
   Reference PDF : 
   Code Writer : hoony
   notation :
       x1 := v [m/s] Speed of Vehicle
       x2 := v^2 [^2/s^2]
       u1 := Fm [N] Motor Traction Force
       u2 := Fb [N] Brake Force
       a  := slope [rad] % Considering only constant road slop
%}

clear; clc 

%% Parameters.
load('Data_mat/Parameter_init.mat');
v_r = 80/3.6; % Tracking Speed [m/s]
a = 0;
u_e = m*(Ca*v_r.^2+g*(Cr*cos(a)+sin(a)));
d_array = [1,10,50,100,200];
v_array = 10:10:100;

%% Dynamic Equations.
% Non-Lear Dynanics
f = @(x1,u,a) 1/x1*(u(1)./m+u(2)./m-Ca*x1.^2-g*(Cr*cos(a)+sin(a)));

% Linearization Dynanics
A = [-2*Ca 0; 0 -2*Ca]; B=[1/(m*v_r) 1/(m*v_r);2/m 2/m]; D=[2*Ca-u_e/(m*v_r);-2*g*(Cr*cos(a)+sin(a))];

%% DATA Generation & Analysis.
for j=1:5
    d = d_array(j);
    Ad = expm(A*d); Bd = A\(Ad-eye(2))*B;
    for i=1:10
        v0 = (10*i)/3.6;
        V0 = [v0;v0^2];
        u = [4000;0];
        [t,xn1] = ode45(@(t,xn1) 1/xn1*(u(1)./m+u(2)./m-Ca*xn1.^2-g*(Cr*cos(a)+sin(a))),[0 d],v0);
%         plot(t,xn1,'r-o');
%         hold on
        vn = Ad*V0+Bd*u;
%         plot([0,d],[v0,vn(1,1)],'b--');
        res_vnl(i,j) = xn1(end);
        res_vl(i,j) = vn(1,1);
        res_v2(i,j) = vn(2,1);
    end
end
error = abs(res_vnl-res_vl)*3.6;
figure(2)
surf(d_array,v_array,error);
title(['Reference Speed : ', num2str(v_r*3.6), 'kph, ','Input : ', num2str(sum(u)), '[N]' ]);
xlabel('Sample Distance [m]')
ylabel('Speed [kph]')
zlabel('Abs Err [kph]')

figure(3)
% X = categorical({'RMS'});
% X = reordercats(X,{'RMS'});
% Y = [rms1,rms2];
b1 = bar(v_array,error);
title(['Reference Speed : ', num2str(v_r*3.6), 'kph, ','Input : ', num2str(sum(u)), '[N]' ]);
ylabel('Abs Err [kph]');
xlabel('Speed [kph]')
legend('1','10','50','100','200')
figure(4)
X = categorical({'1','10','50','100','200'});
X = reordercats(X,{'1','10','50','100','200'});
b2 = bar(X,error');
title(['Reference Speed : ', num2str(v_r*3.6), 'kph, ','Input : ', num2str(sum(u)), '[N]' ]);
ylabel('Abs Err [kph]');
xlabel('Sample Distance [m]')