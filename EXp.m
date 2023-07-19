% © Rahul Kala, IIIT Allahabad, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% Please cite the work in all materials as: R. Kala (2014) Code for Robot Path Planning using Artificial Potential Fields, Indian Institute of Information Technology Allahabad, Available at: http://rkala.in/codes.html
clc;
close all;
clear all;
map=int16(im2bw(imread('Arena.bmp'))); % input map read from a bmp file. for new maps write the file name here
t=1;
rectangle('position',[1 1 size(map)-1],'edgecolor','k')
pathLength=0; 
t=t+1;
tic;
imshow(map==1);

source=[95 454 ;
    137 453;
    303 453;
    341 453;

    ];
        
        %444 243];
robot_CurrentPoses=[source];
goals=[160 60;
    380 170;

    100 61;
    380 140;

    308 60;
    400 340;

    360 60;
    400 164;
    ];

%     109 32;
%     404 300;
% 
%     148 23;
%     67 227;];

% goals=[374 39;
%     132 144;
%     
%     266 377;
%     318 45;
%     
%     117 55;
%     119 346];
robotSize=9;
robots=[];
num_Robots=size(source,1);
colors=rand(3,num_Robots);
for h=1:num_Robots
    robots=[robots; RobotS(source(h,:),robotSize,1,10,map,15,colors(:,h))];
end

taskqueue=Queue();
tasks_status=[];
num_tasks=size(goals,1)/2;
for i=1:num_tasks
    task_=Task(goals(2*i-1,:),goals(2*i,:),2);
    taskqueue.enqueue(task_);
    tasks_status = [tasks_status; task_];
end
all_done=false;

task=0;tt=0;
hold on;
scatter(source(1,2),source(1,1),'green','filled');
scatter(source(2,2),source(2,1),'red','filled');
scatter(source(3,2),source(3,1),'cyan','filled');
scatter(source(4,2),source(4,1),'magenta','filled');
 %scatter(source(5,2),source(5,1),"MarkerFaceColor","#D95319");

scatter(goals(1,2),goals(1,1),'green');
scatter(goals(2,2),goals(2,1),'green');
scatter(goals(3,2),goals(3,1),'red');
scatter(goals(4,2),goals(4,1),'red');
scatter(goals(5,2),goals(5,1),'cyan');
scatter(goals(6,2),goals(6,1),'cyan');
scatter(goals(7,2),goals(7,1),'magenta');
scatter(goals(8,2),goals(8,1),'magenta');
% scatter(goals(9,2),goals(9,1),"MarkerFaceColor","#D95319");
% scatter(goals(10,2),goals(10,1),"MarkerFaceColor","#D95319");
max_iter=1000000;
while ~all_done
    if ~taskqueue.isempty()
        if task==0
            task=taskqueue.dequeue();
        end
        mini_robo=1;
        for robot_number=1:num_Robots
            value=robots(robot_number).getTaskEstimate(task)
            if robots(mini_robo).getTaskEstimate(task)<0
                mini_robo=robot_number;
            else
            if value<robots(mini_robo).getTaskEstimate(task) && value~=-1
                mini_robo=robot_number;
            end
            end
        end
        if robots(mini_robo).getTaskEstimate(task)~=-1
            display("Task_assigned to ");
            mini_robo
            robots(mini_robo).addTask(task);
            task=0;
        end
    end

    for e=1:num_Robots
        robot=robots(e);
        robot.updatePose(robot_CurrentPoses);
        robot_CurrentPoses(e,:)=robot.currentPosition;
        hold on;
        plotRobot(robot.currentPosition,robot.currentDirection,robot.map,robot.robotHalfDiagonalDistance,robot.Color);
        %error('collission recorded');
        %end
        M(t)=getframe;
    end
    num_done_tasks=0;
    for i=1:num_tasks
        if tasks_status(i).TaskStatus~="Completed"
            break;
        end
        num_done_tasks=num_done_tasks + 1;
    end
    if num_done_tasks==num_tasks
        all_done=true;
    end
    tt = tt+1;
if tt==max_iter
    break;
end
end







%b=Block([250 250]);
%b.updateBlock([])
%b=Block([250 250]);

pathFound_=[0 0];

while nnz(pathFound_)~=size(robots,1)
    %display(size(robots,1))
    for e=1:size(robots,1)
        robot=robots(e);
        robot.updatePose(robot_CurrentPoses);
        robot_CurrentPoses(e,:)=robot.currentPosition;
        hold on;
        plotRobot(robot.currentPosition,robot.currentDirection,robot.map,robot.robotHalfDiagonalDistance);
        %error('collission recorded');
        %end
        M(t)=getframe;
        if size(robot.goals,1)==0
            pathFound_(e)=1;
        end

    end
    t=t+1;
end

fprintf('processing time=%d \nPath Length=%d \n\n', toc)%,pathCost); 
