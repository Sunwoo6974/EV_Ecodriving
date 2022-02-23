function plotvalue(X,Y,Z,table,stage,Sh_mid,Vh_mid,Sf_mid)

slice(X,Y,Z,table,Sh_mid,Vh_mid,Sf_mid)
ylabel('Host Vehicle location [m]')
xlabel('Host Vehicle Speed [m/s]')
zlabel('Frontal Vehicle location [m]')
title([num2str(stage),'-th stage'])
colorbar;