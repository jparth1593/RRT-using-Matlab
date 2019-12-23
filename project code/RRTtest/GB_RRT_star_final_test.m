
clear;
%clc;
close(findobj('type','figure','name','RRT basic'));
close(findobj('type','figure','name','RRT growing'));
format longG
%% 
% define boundry and area.

height = 1000;
width = 1000;
figure('name', 'RRT star');
hold on
axis ([0 width 0 height]);
%% 
% 
% 
% define starting and end point and iteration
%%
qstart = [50,500];  
qgoal = [900,500];
iterations = 1500;
costideal = eudist(qstart,qgoal);
disp(costideal)
slop = abs(qgoal(2)-qstart(2))/(qgoal(1)-qstart(1));
qcenter = [qstart(1)+(((qgoal(1)-qstart(1))/costideal)*(costideal/2)),qstart(2)+(((qgoal(2)-qstart(2))/costideal)*(costideal/2))];
%define OBSTACLE

%OBSTACLE 1
obs1.x=[200 600];
obs1.y=[350 650];

%OBSTACLE 2
%obs2.x=[];
%obs2.y=[];
obs2.x = [0 0];
obs2.y = [0 0];

%create obstacle1
x1box=obs1.x([1 1 2 2 1]);
y1box=obs1.y([1 2 2 1 1]);

%create obstacle2
x2box=obs2.x([1 1 2 2 1]);
y2box=obs2.y([1 2 2 1 1]);

obs1area = abs(obs1.x(1)-obs1.x(2))*abs(obs1.y(1)-obs1.y(2));
obs2area = abs(obs2.x(1)-obs2.x(2))*abs(obs2.y(1)-obs2.y(2));

totalarea = height*width;
maxfree = abs(totalarea - (obs1area+obs2area));
ddimension = 2;
unitballvolume = 2*pi;

% define tree edge, node and vertice
% 
% vertecies: An array of Vertrcies' coordinates, sorted by the generated 
% order

vertecies = qstart;
vert_count = 1;
qtree(vert_count,:)=num2cell(vert_count,[1 2]);
sol_tree_count = 1;
sol_tree_log(sol_tree_count,:) =num2cell(sol_tree_count,[1 2]); 
sol_tree_cost(sol_tree_count,:) = 0;
%vertecies.cost = 0;

eps=(height*3)/100;
eps1=(height*5)/100;
val = 0; 
cmin(vert_count,:)=0;
solcost = 0;
soltree = qstart; 
qnew_cost = 0;
cmax = 0;
% 
% 
% Edges:starting and ending of each edges, note edges(i) corresponds to vertecies(i+1), 
% because vertecies(1) is the original point

edges.x = zeros(iterations,2);
edges.y = zeros(iterations,2);
edge_count = 0;
%% 
% 
% 
% ind_nearest: index to the nearest point. vertecies(ind_nearest(i)) is the 
% closetest vertex to vertecies(i+1)
%%
ind_nearest = zeros(iterations,1);
qnew = [0 0];
colli = 0;
i=1;
randcount=1;
optimumtolarance = 0;
%%
% define variables for optimization


%% 
% define figure and hold
%%
hold on;
mapshow(x1box,y1box,'DisplayType','polygon','LineStyle','none');
mapshow(x2box,y2box,'DisplayType','polygon','LineStyle','none');
scatter(qstart(1), qstart(2), 45, 'o','r','filled'); hold on;
scatter(qgoal(1), qgoal(2), 45, 'o','r','filled'); hold on;
%plot(qstart(1),qstart(2),'r*');
%plot(qgoal(1),qgoal(2),'r*');

%figure('name', 'RRT growing');
tic;
%%
while i <= iterations
    
    [x_rand,y_rand] = samplepoint(height,width,cmax,costideal,slop,qcenter);
  %  x_rand = width*rand();    % random point generation
  %  y_rand = height*rand();
    temp.rand= [x_rand,y_rand]; %save temporary random point
    randcount=randcount+1;
   % drawnow
    %scatter(x_rand,y_rand,30,'filled');
   %if random point is goal point this condition is not consider in this
   %programme yet
    
    % find nearest point using KD-Tree method
    ind_nearest(i) = knnsearch(vertecies, [x_rand,y_rand]);
    qnearest = [vertecies(ind_nearest(i),1),vertecies(ind_nearest(i),2)];
    
    qnew_cost1 = eudist(qnearest,[x_rand,y_rand]);
    qnew_goal_cost = eudist(qgoal,qnearest);
    %find new point using steer funcrtion or BVP
    %find new point towards goal if no obstacle eps1>eps
    %if there is obstacle eps>eps1
    %else use only eps
    %A goabbiasing function
    colli_random = collision(x1box,y1box,x2box,y2box,[x_rand,y_rand],qnearest);
    if colli_random == 0
         %this is tem new point
        qnew =  gbsteerfn(temp.rand(1),temp.rand(2),qnearest(1),qnearest(2),qnew_cost1,qnew_goal_cost,qgoal,eps,eps1);
       colli = collision(x1box,y1box,x2box,y2box,qnew,qnearest);
            if colli == 1
                % valus of eps and eps1 are inter change to not stay in local minima
                qnew =  gbsteerfn(temp.rand(1),temp.rand(2),qnearest(1),qnearest(2),qnew_cost1,qnew_goal_cost,qgoal,eps1,eps);
                colli1 = collision(x1box,y1box,x2box,y2box,qnew,qnearest);
                if colli1 == 1
                   %use normal steer function
                   qnew =  steerfn(temp.rand(1),temp.rand(2),qnearest(1),qnearest(2),qnew_cost1,eps1); 
                   colli2 = collision(x1box,y1box,x2box,y2box,qnew,qnearest);
                   if colli2 == 1
                       continue
                   end
                end
            end
    else
        qnew =  steerfn(temp.rand(1),temp.rand(2),qnearest(1),qnearest(2),qnew_cost1,eps1); 
        colli3 = collision(x1box,y1box,x2box,y2box,qnew,qnearest);
        if colli3 == 1
        continue
        end
    end
    
            
            
    %check collision
    %colli = collision(x1box,y1box,x2box,y2box,qnew,qnearest);
    
   % if colli_check == 1
   %     continue 
   % end
    
    %calculate radius for rangesearch
    Rrrt = 2*((1+(1/ddimension))^(1/ddimension))*((maxfree/unitballvolume)^(1/ddimension));
    dynamicradius = Rrrt*((log(vert_count)/vert_count)^(1/ddimension));
   %optimization in edge developement
   
        qnew_cost = eudist(qnearest,qnew);
        try1 = abs((dynamicradius-eps)/2);
        qrange1=rangesearch(vertecies,qnew,max(dynamicradius,eps));
        %qrange1=rangesearch(vertecies,qnew,try1);
        qrange=cell2mat(qrange1);
        txxx=[vertecies(qrange,1),vertecies(qrange,2)];
        cmin1 = cmin(ind_nearest(i)) + qnew_cost;
        qmin1 = qnearest;
        txx = length(qrange);
        %rewire process
        for k = 1:length(qrange)
          %  txxz = eudist([vertecies(qrange(k),1),vertecies(qrange(k),2)],qnew); 
           % tzxzz = qrange(k);
            cnew = cmin(qrange(k)) + eudist([vertecies(qrange(k),1),vertecies(qrange(k),2)],qnew);
            if cnew < cmin1
                tzz = 1;
               if 0 == collision(x1box,y1box,x2box,y2box,qnew,[vertecies(qrange(k),1),vertecies(qrange(k),2)])
                   qmin1 = [vertecies(qrange(k),1),vertecies(qrange(k),2)];
                   cmin1 = cnew;
                   ind_nearest(i) = qrange(k);
               end
            end
        end
        
            
            
    %vertecies(vert_count+1,:) = [x_rand, y_rand];
    vertecies(vert_count+1,:) = qnew;
    
    edges.x(edge_count+1,:) = [qmin1(1), qnew(1)];
    edges.y(edge_count+1,:) = [qmin1(2), qnew(2)];
    edge_count = edge_count + 1;
    qtree(vert_count+1,:) = addtreenode(vert_count,qtree,ind_nearest(i));
    cmin(vert_count+1,:) = cmin1;
    
    for j = 1:length(qrange)
        cnear = cmin(qrange(j));
        cmin2 = cmin1 + eudist(qnew,[vertecies(qrange(j),1),vertecies(qrange(j),2)]);
        if cmin2 < cnear
            if 0 == collision(x1box,y1box,x2box,y2box,qnew,[vertecies(qrange(j),1),vertecies(qrange(j),2)])
               % ind_nearest(i) = vert_count+1;
                qnewrewire = [vertecies(qrange(j),1),vertecies(qrange(j),2)];
                %vert_count_rewire = qrange(j);
                qtree(qrange(j),:) = addtreenode(qrange(j)-1,qtree,vert_count+1);
                cmin(qrange(j),:) = cmin2;
               % rw = cell2mat(qtree(qrange(j)));
                %rw1 = cell2mat(qtree(vert_count+1));
                %rw1(length(rw1)+1) = qrange(j);
            end
        end
    end
    
    
    
    % find final path
    
    goalcost = eudist (qnew,qgoal);
    
    
    if goalcost <= eps
        solcost = cmin(vert_count+1,:) + goalcost;
        soltree = createfinalpath(vertecies,qtree,vert_count,qgoal);
        %final_tree_count = 1;
        sol_tree_log(sol_tree_count,:) = num2cell(soltree,[1 2]);
        sol_tree_cost(sol_tree_count,:) = solcost;
        sol_tree_count = sol_tree_count + 1;
        
        [min_cost_tree,min_cost_index] = min(sol_tree_cost);
        soltree = cell2mat(sol_tree_log(min_cost_index));
        cmax= min_cost_tree;
        
        optimumtolarance =  ((costideal*15)/100)+costideal;
         if cmax <= optimumtolarance
           %disp(optimumtolarance)
           break;
         end
    end
    
    
  %  hold on
  %  drawnow
  %  line(finaltree(:,1),finaltree(:,2),'LineWidth',1,'Color','g');
 %   scatter(vertecies(i,1), vertecies(i,2), 5,linspace(1,10,length(vertecies(i,1))),'filled'); hold on;
 %   plot(edges.x', edges.y');
    
    
    
   
    vert_count = vert_count + 1;
    i=i+1;
end


%if not find final route 

if cmax == 0
    ind_nearest1 = knnsearch(vertecies,qgoal);
    qnearest = [vertecies(ind_nearest1,1),vertecies(ind_nearest1,2)];
    finalcolli = collision(x1box,y1box,x2box,y2box,qgoal,qnearest);
    if finalcolli == 0

    goalcost = eudist (qnearest,qgoal);
    cmax = cmin(ind_nearest1,:) + goalcost;
    soltree = createfinalpath(vertecies,qtree,ind_nearest1-1,qgoal);
    else
         disp('no nearby points')
    end
end

% path finishing
%if cmax >=1
 %  soltree = pathfinsh(soltree);
%end

disp(cmax)

toc;
clear i x_rand y_rand edge_rand
disp(randcount)
disp(vert_count)
%disp(cmax)
hold on
%mapshow(x1box,y1box,'DisplayType','polygon','LineStyle','none');
%mapshow(x2box,y2box,'DisplayType','polygon','LineStyle','none')
%scatter(qstart(1), qstart(2), 45, 'o','r','LineWidth',1); hold on;
%scatter(qgoal(1), qgoal(2), 45, 'o','r','LineWidth',1); hold on;
scatter(vertecies(:,1), vertecies(:,2), 1,linspace(1,10,length(vertecies(:,1))),'filled'); hold on;
plot(edges.x', edges.y','LineWidth',0.01);
line(soltree(:,1),soltree(:,2),'LineWidth',2,'Color','r');

