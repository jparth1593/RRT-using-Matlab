function qtreeout = addtreenode( vc,qt,in )
% add new nodes in TREE with minimum cost
qt1=cell2mat(qt(in));

qt1(length(qt1)+1)=vc+1;

qtreeout=num2cell(qt1,[1 2]);
end

