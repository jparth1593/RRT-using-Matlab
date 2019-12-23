function co = collision( x1box,y1box,x2box,y2box,qnear,nearest )
%collision check 
%   create temporary edge between nearest point in vertecies and new point
tempedgex = [nearest(1),qnear(1)];
tempedgey = [nearest(2),qnear(2)];
%plot(tempedgex',tempedgey');
%check insersection point between edges of the obstacle and new point to
%nearest point edge

[x1i,y1i] = polyxpoly(tempedgex,tempedgey,x1box,y1box);
[x2i,y2i] = polyxpoly(tempedgex,tempedgey,x2box,y2box);

csi1 = size([x1i,y1i]);
csi2 = size ([x2i,y2i]);
%a=csi1(1);
%b=csi2(1);
if (csi1(1)==0 && csi2(1) ==0)
    co = 0;
else
   co= 1;
end

end

