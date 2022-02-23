function [copya, copyb, copyc] = Jinterp(a,b,c)

%a = [1:5]';
%b = [1:6];
%c = [1:7]';
na = size(a,1);
nb = size(b,2);
nc = size(c,1);

copya = repmat(a,1,nb,nc);
copyb = repmat(b,na,1,nc);
copyc = repmat(c,1,nb,na);
copyc = permute(copyc,[3 2 1]);

end