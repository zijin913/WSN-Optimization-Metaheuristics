%%%%%Particle Swarm Optimization%%%%%%%%%%

%%It is the most basic code of PSO 
%hence makes it easy to apply on atleast unconstrained problems
%% Double Boundary Control Strategy 
%It is used to have some variation in the population which has violated the
%boundary consitions
%Method I 
% If it violates it becomes equal to boundary value
%Method II 
% If violated the boundary condition a value between upper and lower bound
% is chosen 

%% It is best for Newbies 
% The code is vectorized and Hence makes it even faster.
% Syntax pso_nb('filename',popsize,dim,low,up,epoch)
% popsize - It is the number of swarms or basically the total population 
% In swarm intelligence a population of maximum 80 works for every problem
% dim - It refers to the number of variables or the dimensions of the search space
% low - A column matrix depicting the lower boundary for each variable
% up - upper boundary level
% epoch - The max number of iterations.

%% Example
% Type this in your command window :-
% low = [-10;-10;-10];
% up = low+20;
% pso_nb('rosenbrock',40,3,low,up,10e+6);
% Whenever you feel the minimum value has been attained Just Press ---> Ctrl+C
% The globalminimum value and globalminimizer will be there in your workspace

% PSO can also be checked for different benchmark functions - Ackley, Rastrigin
% Grienwank and FoxHoles. Only in the case of FoxHoles change dim = 2
%% code for PSO
function pso_nb(fnc,popsize,dim,low,up,epoch)


%% Initialization and Poulation Generation 
pop=GeneratePopulation(popsize,dim,low,up); 
fitnesspop=feval(fnc,pop);
PBest=pop;
PBest_value=fitnesspop;
[GBest_value,avain]=min(fitnesspop);
for k=1:popsize
GBest(k,:)=pop(avain,:);
end

v=zeros(popsize,dim);
c1=1;
c2=2;
w=2;


for i=1:epoch
   
    mp=1;   % working on hybridization of PSO hence mp variable has been used
    vel = w.*v + mp.*((c1*rand).*(PBest-pop)+(c2*rand).*(GBest-pop));
    offspring = vel + pop;
         
    % BOUNDARY CONTROL IS ALWAYS NECESSARY 
    offspring=BoundaryControl(offspring,low,up);
    
        
    % Cost Value of offspring
    fitness_offspring=feval(fnc,offspring);
    % updating the value of PBest and GBest 
    ind =  fitness_offspring<PBest_value;
    PBest(ind,:)=offspring(ind,:);
    PBest_value(ind,:)=fitness_offspring(ind,:);
    [pop_best,ind1]=min(fitness_offspring);
    if pop_best<GBest_value
        for j=1:popsize
        GBest(j,:)=offspring(ind1,:);
        end
               
        GBest_value=pop_best;
    end
    
    % the new co-ordinates and the velocity
    pop = offspring; 
    v = vel;
    assignin('base','globalminimizer',GBest(1,:));
    assignin('base','globalminimum',GBest_value);
    fprintf('PSO_byBhanu|%5.0f -----> %9.16f\n',i,GBest_value);

end
return

 
