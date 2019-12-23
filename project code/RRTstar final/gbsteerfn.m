function a1=gbsteerfn(x_rand,y_rand,qn1,qn2,qnc,qngc,xgoal,ep,ep1)
    N = [0 0];
    xgoal_x = xgoal(1);
    xgoal_y = xgoal(2);
   % N1=[temp.rand;temp.nearest
    %find step size towards qgoal point 
    if qnc>=eps
        %temp1= dist(temp.rand(1),temp.nearest(1));
       % a = x_rand-qn1;
       % a2 = y_rand-qn2;
        N(1) = abs(qn1 + (((x_rand-qn1)/qnc)*ep)+(((xgoal_x-qn1)/qngc)*ep1));
        N(2) = abs(qn2 + (((y_rand-qn2)/qnc)*ep)+(((xgoal_y-qn2)/qngc)*ep1));
        
    else
        N(1)=x_rand;
        N(2)=y_rand;
    end
    a1= [N(1),N(2)];
end
    
       