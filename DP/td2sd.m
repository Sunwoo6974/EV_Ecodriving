% Function that makes speed profile in spartial domain, using time and
% speed data

% 먼저 fv_t, fv_v를 불러들인다음 t에 따른 s를 뽑자
% 그다음에 linear interpolation
% linear interpolation 된 결과 가지고 1m단위로 뽑아주자 몇미터까지? 데이터 한계까지 뽑아주자

function fv_s = td2sd(fv_v)
%disp("shit")
%size(fv_v,1)
fv_s = zeros(size(fv_v,1)-1,1);
s_cum = 0;

for i = 1:size(fv_v,1)-1
    s_now = (fv_v(i,1) + fv_v(i+1,1))/2;
    s_cum = s_cum + s_now;
    fv_s(i,1)=s_cum;
end

fv_s = [0;fv_s];
fv_s(50,1);
end