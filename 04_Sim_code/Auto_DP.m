%% SPiT Automatic
% 220121
% Sunwoo Kim

clear; clc;
% 이게 뭐하는 코드냐면 매번 내가 parameter 바꿔가면서 dp 데이터 뽑기 귀찮으니까 그냥 한번에 몇개씩

qu_9 = [0.1 1 1];
qu_10 = [0.1 0.5 0.5];
qu_11 = [0.1 0.2 0.2];
qu_12 = [0.1 0.1 0.1];
Vmax = 40;
travel_time = 55;
travel_dist = 500;
theta = [1e3 1e-6 0.5];
fname9 = 'spit9.mat';
fname10 = 'spit10.mat';
fname11 = 'spit11.mat';
fname12 = 'spit12.mat';

%Auto_SPiT_CF_DP_Backward(qu_9,Vmax,travel_time,travel_dist,theta,fname9)
%Auto_SPiT_CF_DP_Backward(qu_10,Vmax,travel_time,travel_dist,theta,fname10)
%Auto_SPiT_CF_DP_Backward(qu_11,Vmax,travel_time,travel_dist,theta,fname11)
Auto_SPiT_CF_DP_Backward(qu_12,Vmax,travel_time,travel_dist,theta,fname12)