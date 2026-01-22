%%to  generate the population
function pop=GeneratePopulation(popsize,dim,low,up)
pop=ones(popsize,dim);
for i=1:popsize
    for j=1:dim
        pop(i,j)=rand*(up(j)-low(j))+low(j);
    end   
end
% for i=1:popsize
%         ch1=round(pop(i,26));
%         ch2=round(pop(i,25));
%         if (ch1-ch2)<3
%             if ch2<22
%                 pop(i,26)=ch2+3+rand*(19-ch2);
%       
%             end
%             if ch2>21
%                 pop(i,25)=21*rand;
%                 pop(i,26)=round(pop(i,25))+3;
%               
%             end
%         end
%         assignin('base','firstpop',pop);
% end
end
