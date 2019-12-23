function ftree = createfinalpath(ver,qt,vc,qg )
%create final path
qt1=cell2mat(qt(vc+1));
%fedx1 = zeros(length(qt1)+1,2);
%fedx1 = zeros(length(qt1)+1,2);


    for j=1:length(qt1)
        qt2(j,:) = [ver(qt1(j),1),ver(qt1(j),2)];
        
    end
    qt2(length(qt2)+1,:) = qg;
    
    ftree = qt2;

end

