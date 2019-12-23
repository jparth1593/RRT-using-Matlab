function a1=steerfn(x_rand,y_rand,qn1,qn2,qnc,eps)
    N = [0 0];
   % N1=[temp.rand;temp.nearest
    %find step size towards qrandom point
    if qnc>=eps
        %temp1= dist(temp.rand(1),temp.nearest(1));
        N(1) = abs(qn1 + ((x_rand-qn1)*eps)/qnc);
        N(2) = abs(qn2 + ((y_rand-qn2)*eps)/qnc);
    else
        N(1)=x_rand;
        N(2)=y_rand;
    end
    a1= [N(1),N(2)];
end
    
       