%{
fv_cycle = readtable('fv_200.csv'); % 99
fv_timetemp = table2array(fv_cycle(:,1));
fv_v = table2array(fv_cycle(:,2))/3.6; % [km/h] to [m/s]
fv_s = td2sd(fv_v); % frontal vehicle location / spatial domain
%}



aa = magic(3)
bb = [1 2 3;4 5 6;7 8 9]

cc1 = (aa < 4 | aa > 7)
cc2 = ((aa < 4) | (aa > 7))

cc1==cc2
