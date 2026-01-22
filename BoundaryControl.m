function pop=BoundaryControl(pop,low,up)
%% Double boundary control strategy
% Strategy can be changed accordingly
[popsize,dim]=size(pop);
for i=1:popsize
    for j=1:dim                
        k=rand<rand; 
        if pop(i,j)<low(j), if k, pop(i,j)=low(j); else pop(i,j)=rand*(up(j)-low(j))+low(j); end, end        
        if pop(i,j)>up(j),  if k, pop(i,j)=up(j);  else pop(i,j)=rand*(up(j)-low(j))+low(j); end, end        
    end
end
return