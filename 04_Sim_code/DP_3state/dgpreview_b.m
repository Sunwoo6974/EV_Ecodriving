function J_dg = dgpreview_b(hv_vnext,hv_tnext,hv_snext,fv_v,fv_t,fv_s,Tqu)
%dg_temp = zeros(size(hv_snext));

r = find(abs(hv_tnext*Tqu - fv_t) < 0.001);
dg_temp = fv_s(r) - hv_snext;


safe_dg1 = (hv_vnext.*(hv_vnext>16.6666))*3.6; % 60km/h 넘길때
safe_dg2 = (hv_vnext-(15/3.6)).*((hv_vnext<16.6666) & (hv_vnext>=(15/3.6)))*3.6; % 15km/h ~ 60km/h
%safe_dg3 = (hv_vnext-(15/3.6)).*(hv_vnext<(15/3.6))*0; % 15km/h 이하
safe_dg3 = 1*(hv_vnext<(15/3.6)); % 15km/h 이하면 1m?
safe_dgt1 = safe_dg1+safe_dg2+safe_dg3;
%safe_dg = repmat(safe_dgt,size(J_dg,1),1);
safe_dgt2 = safe_dgt1'; % 131 x 1
safe_dg = repmat(safe_dgt2,1,size(hv_snext,2)); % 131 x 11

%safe_dg = safe_dgt2;

%J_dg = inf*(safe_dg-J_dg > 0);
%J_dgtemp1 = ~(dg_temp < safe_dg & dg_temp > safe_dg-25); % Backward Step #8~9 //
%J_dgtemp1 = ((dg_temp < 2) | (dg_temp > 100)); % Backward Step #8~9 //
J_dgtemp1 = (dg_temp < 20);% | (dg_temp > 30)); % Backward Step #8~9 // safe dg 말고 그냥 순수 거리차만 가지고 함

J_dg = zeros*J_dgtemp1; % 여기까지는 okay임

%J_dg(dg_temp<0)=1e10; % Host가 더 빨리가면 inf
J_dg(J_dgtemp1==1)=inf; % Backward Step #8~9
%disp("shit");