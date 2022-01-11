function est_dg = dgpreview(vnext,tnext,snext,fv_v,fv_t,fv_s)
est_dg = zeros(size(vnext));


%{
for ix = 1:size(tnext,1)
    for iy = 1:size(tnext,2)
        for iz = 1:size(fv_t,1)
            %ix, iy, iz
            if tnext(ix,iy) == fv_t(iz)
                est_dg = fv_s(iz) - snext(ix,iy)
                disp("shit")
            end
        end
    end
end
%}

safe_dg1 = (vnext.*(vnext>=16.6666))*3.6;
safe_dg2 = (vnext-(15/3.6)).*(vnext<16.6666)*3.6;
safe_dg = safe_dg1+safe_dg2;

dg1 = (vnext.*(vnext>=16.6666))*3.6;
dg2 = (vnext-(15/3.6)).*((vnext<16.6666) & (vnext>=(15/3.6)))*3.6;
dg3 = (vnext-(15/3.6)).*(vnext<(15/3.6))*0;
safe_dg = dg1+dg2+dg3;


if est_dg < safe_dg
    est_dg = inf;
end

