function J_dg = dgpreview(hv_vnext,hv_tnext,hv_snext,fv_v,fv_t,fv_s)
J_dg = zeros(size(hv_snext));

%{
for ift = 1:size(fv_t,1)
   [r,c] = find(abs(hv_tnext - fv_t(ift)) < 0.001);
   J_dg(r,c) = fv_s(ift) - hv_snext;
end % Backward Step #7
%}

r = find(abs(hv_tnext/10 - fv_t) < 0.001);
J_dg = fv_s(r) - hv_snext;

safe_dg1 = (hv_vnext.*(hv_vnext>=16.6666))*3.6;
safe_dg2 = (hv_vnext-(15/3.6)).*((hv_vnext<16.6666) & (hv_vnext>=(15/3.6)))*3.6;
safe_dg3 = (hv_vnext-(15/3.6)).*(hv_vnext<(15/3.6))*0;
safe_dgt1 = safe_dg1+safe_dg2+safe_dg3;
%safe_dg = repmat(safe_dgt,size(J_dg,1),1);
safe_dgt2 = safe_dgt1'; % 131 x 1
safe_dg = repmat(safe_dgt2,1,size(hv_snext,2)); % 131 x 11

%J_dg = inf*(safe_dg-J_dg > 0);
J_dg = 5*(J_dg < safe_dg | J_dg > safe_dg-25); % Backward Step #8~9 // 지금 여기가 문제있다. why? 차가 너무 빨
J_dg(isnan(J_dg))=0; % Backward Step #8~9
