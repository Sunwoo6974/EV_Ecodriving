function [a,h,x] = gen_road(road_st,N,Ss)
%% Generate Road slope, in other word Road Altitude
%{
   Code Writer : hoony.
   Road Scenario
     0 : Flat slope
     1 : Up slope
     2 : Down slppe
     3 : Flat -> Up.
     4 : Flat -> Down
     5 : Up -> Flat
     6 : Down -> Flat
     7 : UP -> Down
     8 : Down -> Up   
%}
% Standard Level Flat slope
x = 0:Ss:Ss*N-1;
var1 = 300/Ss;
si = fix(N/2)-var1; % index
sf = si+2*var1;
var2 = 700/Ss;
si2 = fix(N/2)-var2;
% sf2 = si+2*var2;
sf2 = fix(N/2)+var2;
theta = deg2rad(4);

switch road_st
    case 0
        h = zeros(N,1);
        a = zeros(N-1,1);
    case 1
        h = tan(theta)*x';
        a = round(atan(diff(h)/Ss),7);
    case 2
        h = tan(-theta)*x';
        a = round(atan(diff(h)/Ss),7);
    case 3
        h = [zeros(si,1);tan(theta)/(2*(x(sf)-x(si)))*(x(si+1:sf)'-x(si)).^2;...
            tan(theta)*(x(sf+1:end)-x(sf))'+tan(theta)/(2*(x(sf)-x(si)))*(x(sf)'-x(si)).^2];
        a = round(atan(diff(h)/Ss),7);
    case 4
        h =[zeros(si,1);tan(-theta)/(2*(x(sf)-x(si)))*(x(si+1:sf)'-x(si)).^2;...
            tan(-theta)*(x(sf+1:end)-x(sf))'+tan(-theta)/(2*(x(sf)-x(si)))*(x(sf)'-x(si)).^2];
        a = round(atan(diff(h)/Ss),7);
    case 5
        h = flip([zeros(si,1);tan(-theta)/(2*(x(sf)-x(si)))*(x(si+1:sf)'-x(si)).^2;...
            tan(-theta)*(x(sf+1:end)-x(sf))'+tan(-theta)/(2*(x(sf)-x(si)))*(x(sf)'-x(si)).^2]);
        a = round(atan(diff(h)/Ss),7);
    case 6
        h = flip([zeros(si,1);tan(theta)/(2*(x(sf)-x(si)))*(x(si+1:sf)'-x(si)).^2;...
            tan(theta)*(x(sf+1:end)-x(sf))'+tan(theta)/(2*(x(sf)-x(si)))*(x(sf)'-x(si)).^2]);
        a = round(atan(diff(h)/Ss),7);
    case 7
        h = [tan(theta)*x(1:si2)';...
            tan(theta)/(x(si2)-x(sf2))*(x(si2+1:sf2)'-(x(si2)+x(sf2))/2).^2+tan(theta)*x(si2)-tan(theta)/(x(si2)-x(sf2))*(x(si2)'-(x(si2)+x(sf2))/2).^2;
            tan(-theta)*(x(sf2+1:end)'-x(sf2))+...
            tan(theta)/(x(si2)-x(sf2))*(x(sf2)'-(x(si2)+x(sf2))/2).^2+tan(theta)*x(si2)-tan(theta)/(x(si2)-x(sf2))*(x(si2)'-(x(si2)+x(sf2))/2).^2];
        a = round(atan(diff(h)/Ss),7);
    case 8
        h = -[tan(theta)*x(1:si2)';...
            tan(theta)/(x(si2)-x(sf2))*(x(si2+1:sf2)'-(x(si2)+x(sf2))/2).^2+tan(theta)*x(si2)-tan(theta)/(x(si2)-x(sf2))*(x(si2)'-(x(si2)+x(sf2))/2).^2;
            tan(-theta)*(x(sf2+1:end)'-x(sf2))+...
            tan(theta)/(x(si2)-x(sf2))*(x(sf2)'-(x(si2)+x(sf2))/2).^2+tan(theta)*x(si2)-tan(theta)/(x(si2)-x(sf2))*(x(si2)'-(x(si2)+x(sf2))/2).^2];
        a = round(atan(diff(h)/Ss),7);
end
%plot(h)
end