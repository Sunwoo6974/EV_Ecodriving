%{
fv_cycle = readtable('fv_200.csv'); % 99
fv_timetemp = table2array(fv_cycle(:,1));
fv_v = table2array(fv_cycle(:,2))/3.6; % [km/h] to [m/s]
fv_s = td2sd(fv_v); % frontal vehicle location / spatial domain
%}

%% Host vehicle DP data loading and driving simulation settings
aa = [0:30]
nnn = aa*3.6

aa1 = (aa.*(aa>=16.6666))*3.6;
aa2 = (aa-(15/3.6)).*((aa<16.6666) & (aa>=(15/3.6)))*3.6;
aa3 = (aa-(15/3.6)).*(aa<(15/3.6))*0;
aanew = aa1+aa2+aa3