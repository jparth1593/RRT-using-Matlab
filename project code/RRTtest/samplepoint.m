function [xran,yran,eps,eps1 ] = samplepoint(hi,wi,cmax,cid,slop,qcen,eps,eps1)
%create sampling point based on the first path 
%   use the initial path value to generate normal distribution(gaussian distribution)
% so that limit the state space which help to get more cost effective path
% in less time



 
    
if cmax >= 1
    eps=(cmax*3)/100;
    eps1=(cmax*5)/100;
    rotation = [cos(slop) sin(slop);sin(slop) cos(slop)];
    trandia = cmax/2;
    conjudia = (((((cmax)^2)-((cid)^2))^(1/2))/2)/2;
    ran = randn(1,2);       %create random point
    ran1 = ran*[trandia 0;0 conjudia];      %calculate with 
    
    ran2 = ran1*rotation;
    ran3 = ran2+qcen;
    xran = ran3(1);
    yran = ran3(2);
    
else
    xran = wi*rand();    % random point generation
    yran = hi*rand();
end



end

