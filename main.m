N = 2;
initial_positions = [0;0;0];
initial_positions(:,2)=[0.5;0.5;1];
TaskQueue = Queue();
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);
robots = Robot.empty();
blocks = Block.empty();
for robotid = 1:N
    robots(robotid) = Robot(initial_positions(:,robotid),2);
end
r.get_poses()
r.step()
estimate = [];

steps = 10;
while 1
    if steps > 10
    if task==0
        task = TaskQueue.dequeue;
    end
    for robotid = 1:N
        estimate(robotid) = robots(robotid).getTaskEstimate(task);
    end
    AssigneTaskTo = AssigneTask(estimate);
    if AssigneTaskTo ~= -1
        for i = 1:len(AssigneTaskTo)
            robotid(AssigneTaskTo(i)).addTask(task);
        end
        task=0;
    end
    step = 0;
    end
    Robot_pos = r.get_poses();
    dxu = get_velocities(Robot_pos,robots,blocks);
    r.set_velocities(1:N,dxu);
    r.step();
    saveas(r.figure_handle,'Barchart.png')
end
