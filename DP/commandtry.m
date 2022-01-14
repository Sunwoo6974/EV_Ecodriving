%{
fv_cycle = readtable('fv_200.csv'); % 99
fv_timetemp = table2array(fv_cycle(:,1));
fv_v = table2array(fv_cycle(:,2))/3.6; % [km/h] to [m/s]
fv_s = td2sd(fv_v); % frontal vehicle location / spatial domain
%}


%{
aa = magic(6);
aaorigin = aa
%bb = ones(6,6)

bb = inf*(aa>10)

cc = aa.*bb;
cc(isnan(cc))=1;
aa = aa.*cc
%}

%{
aa = magic(3)
bb = [1 2 3;4 5 6;7 8 9]

cc = (aa-bb < 0)

cci = ~cc
%}

aa = magic(5);
bb = zeros(5);
for i = 1:5
    bb(i,:) = min(aa(i,:));
end