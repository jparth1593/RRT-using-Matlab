
clear;
%clc;
close(findobj('type','figure','name','RRT basic'));
close(findobj('type','figure','name','RRT growing'));

%% 
% define boundry and area.

height = 1000;
width = 1000;
figure('name', 'RRT star');
axis ([0 width 0 height]);
%% 
% 
% 
% define starting and end point and iteration
%%
qstart = [50,50];
qgoal = [750,850];
iterations = 2000;

%define OBSTACLE

%OBSTACLE 1
obs1.x=[0 0];
obs1.y=[0 0];

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


% define tree edge, node and vertice
% 
% vertecies: An array of Vertrcies' coordinates, sorted by the generated 
% order

vertecies = qstart;
vert_count = 1;
qtree(vert_count,:)=num2cell(vert_count,[1 2]);
%vertecies.cost = 0;
format longG
eps=15;
val = 0; 
cmin(vert_count,:)=0;
finalcost = 0;
finaltree = qstart; 
qnew_cost = 0;
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
%%
% define variables for optimization


%% 
% define figure and hold
%%
hold on;
mapshow(x1box,y1box,'DisplayType','polygon','LineStyle','none');
mapshow(x2box,y2box,'DisplayType','polygon','LineStyle','none');
scatter(qstart(1), qstart(2), 45, '*','r','LineWidth',1); hold on;
scatter(qgoal(1), qgoal(2), 45, '*','r','LineWidth',1); hold on;

%figure('name', 'RRT growing');
tic;
%%
while i <= iterations
    
    x_rand = width*rand();    % random point generation
    y_rand = height*rand();
    temp.rand= [x_rand,y_rand]; %save temporary random point
    randcount=randcount+1;
    
    
    % find nearest point using KD-Tree method
    ind_nearest(i) = knnsearch(vertecies, [x_rand,y_rand]);
    qnearest = [vertecies(ind_nearest(i),1),vertecies(ind_nearest(i),2)];
    
    qnew_cost1 = eudist(qnearest,[x_rand,y_rand]);
    %find new point using steer funcrtion or BVP
    qnew =  steerfn(temp.rand(1),temp.rand(2),qnearest(1),qnearest(2),qnew_cost1,eps); %this is tem new point
                       
    %check collision
    colli = collision(x1box,y1box,x2box,y2box,qnew,qnearest);
    
    if colli == 1
        continue
    end
        
     
   %optimization in edge developement
   
   qnew_cost = eudist(qnearest,qnew);
   
   %qrange1=rangesearch(vertecies,qnew,eps+20);
   %qrange=cell2mat(qrange1);
    
    
    %vertecies(vert_count+1,:) = [x_rand, y_rand];
    vertecies(vert_count+1,:) = qnew;
    
    edges.x(edge_count+1,:) = [vertecies(ind_nearest(i),1), qnew(1)];
    edges.y(edge_count+1,:) = [vertecies(ind_nearest(i),2), qnew(2)];
    edge_count = edge_count + 1;
    
    qtree(vert_count+1,:) = addtreenode(vert_count,qtree,ind_nearest(i));
    cmin(vert_count+1,:) = cmin(ind_nearest(i))+qnew_cost;
    
    % find final path
    
    goalcost = eudist (qnew,qgoal);
    
    if goalcost <= eps
        finalcost = cmin(vert_count+1,:) + goalcost;
        finaltree = createfinalpath(vertecies,qtree,vert_count,qgoal);
    end
    
    
%scatter(vertecies(i,1), vertecies(i,2), 5,linspace(1,10,length(vertecies(i,1))),'filled'); hold on;
%plot(edges.x', edges.y');
   vert_count = vert_count + 1;
    i=i+1;
    
end
disp(randcount)


%if not find final route 
if finalcost == 0
    ind_nearest1 = knnsearch(vertecies,qgoal);
    qnearest = [vertecies(ind_nearest1,1),vertecies(ind_nearest1,2)];
    finalcolli = collision(x1box,y1box,x2box,y2box,qgoal,qnearest);
    if colli == 1
        disp('no nearby points')
    end
    goalcost = eudist (qnearest,qgoal);
    finalcost = cmin(ind_nearest1,:) + goalcost;
    finaltree = createfinalpath(vertecies,qtree,ind_nearest1,qgoal);
end
disp(finalcost)

toc;
clear i x_rand y_rand edge_rand
testidealdist = eudist(qstart,qgoal);
disp(testidealdist)
%mapshow(x1box,y1box,'DisplayType','polygon','LineStyle','none');
%mapshow(x2box,y2box,'DisplayType','polygon','LineStyle','none')
%scatter(qstart(1), qstart(2), 45, 'o','r','LineWidth',1); hold on;
%scatter(qgoal(1), qgoal(2), 45, 'o','r','LineWidth',1); hold on;
scatter(vertecies(:,1), vertecies(:,2), 5,linspace(1,10,length(vertecies(:,1))),'filled'); hold on;
plot(edges.x', edges.y');
line(finaltree(:,1),finaltree(:,2),'LineWidth',2,'Color','r');

%plot(finaltree(:,1),finaltree(:,2),'MarkerEdgeColor','k','MarkerFaceColor',[0,0,0]);