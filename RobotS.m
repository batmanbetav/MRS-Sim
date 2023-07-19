classdef RobotS<handle
    properties 
    name;
    RobotStatus=0;
    Taskqueue;
    source=[100 100];
    goals=[];
    robotDirection=pi/8; % initial heading direction
    robotSize=[10 10]; %length and breadth
    robotSpeed=10; % arbitrary units 
    maxRobotSpeed=5; % arbitrary units 
    distanceThreshold=5; % a threshold distace. points within this threshold can be taken as same. 
    maxAcceleration=10; % maximum speed change per unit time
    %maxTurn=10*pi/180; % potential outputs to turn are restriect to -60 and 60 degrees.
    maxTurn=pi/2;
    attractivePotentialScaling=300000; % scaling factor for attractive potential
    repulsivePotentialScaling=300000; % scaling factor for repulsive potential
    minAttractivePotential=0.5; % minimum attractive potential at any point
    map;
    Color;
    %%%%% parameters end here %%%%%
    
    currentPosition;%=source; % position of the centre of the robot
    currentDirection;%=robotDirection; % direction of orientation of the robot
    robotHalfDiagonalDistance;%=((robotSize(1)/2)^2+(robotSize(2)/2)^2)^0.5; % used for distance calculations 
    pathFound=0; % has goal been reached
    pathCost=0;
    SensorRange;
    CurrentTask;
    end
    methods
        function obj=RobotS(source,robotSize,robotSpeed,distanceThreshold,map,SensorRange,Color) 
            obj.source=source;
            obj.currentPosition=source;
            obj.currentDirection=obj.robotDirection;
            obj.robotSize=[robotSize robotSize];
            obj.robotSpeed=robotSpeed;
            obj.robotHalfDiagonalDistance=((obj.robotSize(1)/2)^2+(obj.robotSize(2)/2)^2)^0.5;
            obj.distanceThreshold=distanceThreshold;
            obj.map=map;
            obj.SensorRange=SensorRange;
            obj.goals=[];
            obj.Taskqueue=Queue();
            obj.Color=Color;
        end
        function updatePose(obj,other_coor)
            k=3;

            if obj.RobotStatus == 1
                if size(obj.goals,1)==0
                    obj.goals = [obj.CurrentTask.StartLocation;obj.CurrentTask.EndLocation];
                end
            
                goal=obj.goals(1,:);
                x=int16(obj.Sensor(other_coor,obj.currentPosition));
                obj.map=obj.map.*x;
                i=obj.robotSize(1)/2+1;
                while true
                    x=int16(obj.currentPosition+i*[sin(obj.currentDirection) cos(obj.currentDirection)]);
                    if ~feasiblePoint(x,obj.map),   break; end
                    i=i+1;
                end
            distanceFront=i-obj.robotSize(1)/2; % robotSize(1)/2 distance included in i was inside the robot body 
            
            % calculate distance from obstacle at left
            i=obj.robotSize(2)/2+1;
            while true
                x=int16(obj.currentPosition+i*[sin(obj.currentDirection-pi/2) cos(obj.currentDirection-pi/2)]);
                if ~feasiblePoint(x,obj.map), break; end
                i=i+1;
            end
            distanceLeft=i-obj.robotSize(2)/2;  
            
            % calculate distance from obstacle at right
        
            i=obj.robotSize(2)/2+1;
            while true
                x=int16(obj.currentPosition+i*[sin(obj.currentDirection+pi/2) cos(obj.currentDirection+pi/2)]);
                if ~feasiblePoint(x,obj.map), break; end
                i=i+1;
            end
            distanceRight=i-obj.robotSize(2)/2;  
            
            % calculate distance from obstacle at front-left diagonal
            i=obj.robotHalfDiagonalDistance+1;
            while true
                x=int16(obj.currentPosition+i*[sin(obj.currentDirection-pi/4) cos(obj.currentDirection-pi/4)]);
                if ~feasiblePoint(x,obj.map), break; end
                i=i+1;
            end
            distanceFrontLeftDiagonal=i-obj.robotHalfDiagonalDistance;
            
            % calculate distance from obstacle at front-right diagonal
            i=obj.robotHalfDiagonalDistance+1;
            while true
                x=int16(obj.currentPosition+i*[sin(obj.currentDirection+pi/4) cos(obj.currentDirection+pi/4)]);
                if ~feasiblePoint(x,obj.map), break; end
                i=i+1;
            end
            distanceFrontRightDiagonal=i-obj.robotHalfDiagonalDistance;
            
            % calculate angle from goal
             angleGoal=atan2(goal(1)-obj.currentPosition(1),goal(2)-obj.currentPosition(2));
            
             % calculate distance from goal
             distanceGoal=(sqrt(sum((obj.currentPosition-goal).^2)));
             if distanceGoal<obj.distanceThreshold
                 obj.goals(1,:)=[];
                 if size(obj.goals,1)==0
                    obj.TaskFinished(); 
                 end
             end
             
             % compute potentials
             repulsivePotential=(1.0/distanceFront)^k*[sin(obj.currentDirection) cos(obj.currentDirection)] + ...
             (1.0/distanceLeft)^k*[sin(obj.currentDirection-pi/2) cos(obj.currentDirection-pi/2)] + ...
             (1.0/distanceRight)^k*[sin(obj.currentDirection+pi/2) cos(obj.currentDirection+pi/2)] + ...
             (1.0/distanceFrontLeftDiagonal)^k*[sin(obj.currentDirection-pi/4) cos(obj.currentDirection-pi/4)] + ...
             (1.0/distanceFrontRightDiagonal)^k*[sin(obj.currentDirection+pi/4) cos(obj.currentDirection+pi/4)];
             
             attractivePotential=max([(1.0/distanceGoal)^k*obj.attractivePotentialScaling obj.minAttractivePotential])*[sin(angleGoal) cos(angleGoal)];
             totalPotential=attractivePotential-obj.repulsivePotentialScaling*repulsivePotential;
             
             % perform steer
             preferredSteer=atan2(obj.robotSpeed*sin(obj.currentDirection)+totalPotential(1),obj.robotSpeed*cos(obj.currentDirection)+totalPotential(2))-obj.currentDirection;
             while preferredSteer>pi, preferredSteer=preferredSteer-2*pi; end % check to get the angle between -pi and pi
             while preferredSteer<-pi, preferredSteer=preferredSteer+2*pi; end % check to get the angle between -pi and pi
             preferredSteer=min([obj.maxTurn preferredSteer]);
             preferredSteer=max([-obj.maxTurn preferredSteer]);
             obj.currentDirection=obj.currentDirection+preferredSteer;
             
             % setting the speed based on vehicle acceleration and speed limits. the vehicle cannot move backwards.
             preferredSpeed=sqrt(sum((obj.robotSpeed*[sin(obj.currentDirection) cos(obj.currentDirection)] + totalPotential).^2));
             preferredSpeed=min([obj.robotSpeed+obj.maxAcceleration preferredSpeed]);
             obj.robotSpeed=max([obj.robotSpeed-obj.maxAcceleration preferredSpeed]);
             obj.robotSpeed=min([obj.robotSpeed obj.maxRobotSpeed]);
             obj.robotSpeed=max([obj.robotSpeed 0]);
             
             if obj.robotSpeed==0, error('robot had to stop to avoid collission'); end
             
             % calculating new position based on steer and speed
             newPosition=obj.currentPosition+obj.robotSpeed*[sin(obj.currentDirection) cos(obj.currentDirection)];
             obj.pathCost=obj.pathCost+distanceCost(newPosition,obj.currentPosition);
             obj.currentPosition=newPosition;
             %obj.path=[obj.path obj.currentPosition]
             if ~feasiblePoint(int16(obj.currentPosition),obj.map), error('collission recorded'); end
            end
        end
        
        function h=distanceCost(a,b)
            h = sqrt(sum(a-b).^2);
        end
        function feasible=feasiblePoint(point,map)
            feasible=true;
            % check if collission-free spot and inside maps
            if ~(point(1)>=1 && point(1)<=size(map,1) && point(2)>=1 && point(2)<=size(map,2) && map(point(1),point(2))==1)
                feasible=false;
            end
        end
        function x= Sensor(obj,objects,pos)
            
            x=ones(500);
            %objects=round(objects)
            object = ones(obj.robotSize(1));
            for i=1:length(objects)
                if(norm(pos-objects(i,:))<obj.SensorRange && norm(pos-objects(i,:))>0)
                    x(objects(i,1)-4:objects(i,1)+4,objects(i,2)-4:objects(i,2)+4) = max(0,x(objects(i,1)-4:objects(i,1)+4,objects(i,2)-4:objects(i,2)+4)-object);
                end
            end
        end
        
        function addTask(obj,Task)
           obj.Taskqueue.enqueue(Task);
           obj.startNewTask()
        end

        function startNewTask(obj)
           if obj.RobotStatus == 0
               if ~obj.Taskqueue.isempty()
                  obj.CurrentTask = obj.Taskqueue.dequeue().data;
                  obj.RobotStatus = 1 ;
               end
           end
        end

        function TaskFinished(obj)
           obj.RobotStatus = 0;
           obj.CurrentTask.TaskStatus = "Completed";
           obj.CurrentTask=[];
           obj.startNewTask();
       end

       function estimate=getTaskEstimate(obj,Task)
           eps=0.01;
           if obj.RobotStatus == 1
               %if ~obj.Taskqueue.isempty()
                   estimate = -1;
%                else
%                    estimate = norm(obj.currentPosition-obj.CurrentTask.EndLocation) + norm(Task.data.StartLocation-obj.CurrentTask.EndLocation);
               %end
           else
               estimate = norm(obj.currentPosition-Task.data.StartLocation);
           end
        end

    end
end