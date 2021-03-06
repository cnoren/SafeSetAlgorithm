%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  MAIN CODE IN RSSA              %
%  CHARLES NOREN                  %
%  2021.2                         %
%                                 %
%  ORIGINALLY BY:                 %
%  CHANGLIU LIU                   %
%  2015.5                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% HOUSEKEEPING:
clear;          % Clears the terminal
close all;      % Closes all Figures
addpath('lib'); % Adds a path to all the helper functions
%% Initialization
robot=robust_robotproperty(1);
agent=agentproperty(1);

flag=0;%sign that the game is down
flag1=0;%sign that robot has finished all the task
flag2=0;%sign that human has finished all the task

use_cursor=0;%use defined agent or cursor
standstill_flag = 0% use recorded data, stand still
noviol_flag = 1 % Use some prior defined data
if use_cursor==0
    if standstill_flag ==1
        cp = load('standstill_cursor.mat');
        cp = cp.cp;
    elseif noviol_flag == 1
        cp = load('test.mat');
        cursor_pos_center = cp.cursor_pos_center;
        cursor_pos_URcorner = cp.cursor_pos_URcorner;
        cp = cp.acp;
    end
end
t=1;%count time step
exp_num = 0; % This is for seeding the goals.
exp_rng = rng(exp_num); % This is the actual rng 
num_goals = 100; % This is the number of goals we want
num_goal_counter_robot = 1; % The goal we are on for the robot
num_goal_counter_agent = 1; % The goal we are on for the robot
all_goals = randn(2,num_goals*2)./5;
robo_goals = all_goals(:,1:num_goals);
agent_goals = all_goals(:,(num_goals+1):2*num_goals);

%% initialize the control plot
generate_animation_plot;

%% calibrate the control before the test
if use_cursor == 1
    calibmark.xy = [0.5; 0.5];
    calibmark.handle = plot(calibmark.xy(1),calibmark.xy(2),'o','linewidth',3,'color','b','markersize',14);
    set(calibmark.handle,'XDataSource','calibmark.xy(1)');
    set(calibmark.handle,'YDataSource','calibmark.xy(2)');

    calibmark.xy = [0; 0];
    refreshdata([calibmark.handle],'caller');
    drawnow;
    pause(3)
    cursor_pos_center = get(0,'PointerLocation');

    calibmark.xy = [0.5,0.5];
    refreshdata([calibmark.handle],'caller');
    drawnow;
    pause(3)
    cursor_pos_URcorner = get(0,'PointerLocation');

    calibmark.xy = [10; 10];
    refreshdata([calibmark.handle],'caller');
    drawnow;
end
%% begin testing
set(text1handle,'string','Test runing...')

J = eye(2); % [-1 2; 10 5];
goalhis=[];
t0 = tic();
while flag==0 && t<1001 && num_goal_counter_robot <= num_goals && num_goal_counter_agent <= num_goals

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  robot observe agent        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [robot,agent]=observe(robot,agent,[1,1]);
    %goal inference
    mini=100;ii=1;
    for i=1:agent.nG
        goal=[agent.Goal(1,i);0;agent.Goal(2,i);0];
        dx=robot.obs.xstar(:,end)-goal;
        score=-dx'*robot.const.P1*dx/(dx'*robot.const.P2*dx);
        if score>0 && score<mini
            mini=score;
            ii=i;
        end
    end
    robot.obs.goal(1:2,t)=agent.Goal(:,ii);
            
            
    
%%%%%%%%%%%%%%%%%
%  robot move   %
%%%%%%%%%%%%%%%%%
    if flag1==0
        %robot=solvelqr(robot);
        [flag1,robot]=robust_robotmove(t,robot);
        if flag1~=1
            ie=min([size(robot.x,2),t+1]);
            %plot(robot.x(1,1:ie),robot.x(3,1:ie));
            %drawagent(robot.x(1,ie),robot.x(3,ie),0.5,'k');
            robotposition=robot.wx(1:2:3,ie);
        end
        if flag1==2
            flag1=0;

        end
    end
    

    if flag1==1
        %plot(robot.x(1,:),robot.x(3,:));
        %drawagent(robot.x(1,end),robot.x(3,end),0.5,'k');
        robotposition=robot.wx(1:2:3,end);
    end

    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% update the agent position   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if use_cursor == 1
        cursor_pos_current = get(0,'PointerLocation');
    else
        cursor_pos_current = cp(:,t)';
    end
    agent.cursor_position(:, t) = cursor_pos_current';
    u = (cursor_pos_current - cursor_pos_center)./(cursor_pos_URcorner - cursor_pos_center); % normalized
    agent.x(1,end+1) = u(1)/2;
    agent.x(2,end) = u(2)/2;
    if size(agent.x,2)>1
        agent.x(3,end)=(agent.x(1,end)-agent.x(1,end-1))/robot.delta_t;
        agent.x(4,end)=(agent.x(2,end)-agent.x(2,end-1))/robot.delta_t;
    end
    
    refreshdata([robot.handle, agent.handle, rtrace.handle, atrace.handle],'caller');
    
    t=t+1;
    if t>2
        delete(chandle);
    end
    chandle=capsule(robot.nlink,robot.pos(:,t),robot.x(:,t),[0.07,0.06],'r',0.1,agent.x(:,t),'b');


    if flag1==0 && flag2==0
        refreshdata([rgoal.handle, agoal.handle],'caller');
    end
    
    output=strcat('timestep:',int2str(t),'  score:',int2str(robot.score+agent.score));
    set(text1handle,'string',output)
    drawnow;
    
    if norm(robot.wx(1:2,end)-robot.Goal(:,1))<0.02
        %robot.Goal=randn(2,1)./5; % THIS IS THE ROBOT GOAL
        num_goal_counter_robot = num_goal_counter_robot + 1;
        robot.Goal = robo_goals(:,num_goal_counter_robot);
        if norm(robot.Goal-robot.base(1:2))<norm(robot.l(1)-robot.l(2))
            robot.Goal=(robot.Goal-robot.base(1:2))/norm(robot.Goal-robot.base(1:2))*norm(robot.l(1)-robot.l(2))+robot.base(1:2);
        end
        if norm(robot.Goal-robot.base(1:2))>norm(robot.l(1)+robot.l(2))
            robot.Goal=(robot.Goal-robot.base(1:2))/norm(robot.Goal-robot.base(1:2))*norm(robot.l(1)+robot.l(2))+robot.base(1:2);
        end
    end
    
    agent.goalhis{t}=agent.Goal;
    if norm(agent.x(1:2,end)-agent.Goal(:,1))<0.01
        %agent.Goal=randn(2,1)./5; % THIS IS THE HUMAN GOAL
        num_goal_counter_agent = num_goal_counter_agent + 1;
        agent.Goal = agent_goals(:,num_goal_counter_agent);
        if norm(agent.Goal-robot.base(1:2))<0.25
            agent.Goal=robot.base(1:2)+(agent.Goal-robot.base(1:2))*0.25/norm(agent.Goal-robot.base(1:2));
        end
        if norm(agent.Goal(1))>0.4
            agent.Goal(1)=0.4*agent.Goal(1)/norm(agent.Goal(1));
        end
        if norm(agent.Goal(2))>0.4
            agent.Goal(2)=0.4*agent.Goal(2)/norm(agent.Goal(2));
        end
    end
    
    
    %% THIS IS JUST PLOTTING, HERE!
    if mod(t, 50) == 0
        p1 = plot(robot.pos(1:3:(robot.nlink*3+1),end-1),robot.pos(2:3:(robot.nlink*3+3),end-1),'linewidth',10,'color',[1,0.1,0.1],'markersize',50);
        p2 = plot(robot.pos(1:3:(robot.nlink*3+1),end-2),robot.pos(2:3:(robot.nlink*3+3),end-2),'linewidth',10,'color',[1,0.2,0.2],'markersize',50);
        p3 = plot(robot.pos(1:3:(robot.nlink*3+1),end-4),robot.pos(2:3:(robot.nlink*3+3),end-4),'linewidth',10,'color',[1,0.3,0.3],'markersize',50);
        p4 = plot(robot.pos(1:3:(robot.nlink*3+1),end-8),robot.pos(2:3:(robot.nlink*3+3),end-8),'linewidth',10,'color',[1,0.4,0.4],'markersize',50);
        p5 = plot(robot.pos(1:3:(robot.nlink*3+1),end-12),robot.pos(2:3:(robot.nlink*3+3),end-12),'linewidth',10,'color',[1,0.5,0.5],'markersize',50);
        p6 = plot(robot.pos(1:3:(robot.nlink*3+1),end-16),robot.pos(2:3:(robot.nlink*3+3),end-16),'linewidth',10,'color',[1,0.6,0.6],'markersize',50);
        p7 = plot(robot.pos(1:3:(robot.nlink*3+1),end-20),robot.pos(2:3:(robot.nlink*3+3),end-20),'linewidth',10,'color',[1,0.7,0.7],'markersize',50);
        p8 = plot(robot.pos(1:3:(robot.nlink*3+1),end-24),robot.pos(2:3:(robot.nlink*3+3),end-24),'linewidth',10,'color',[1,0.8,0.8],'markersize',50);
        p9 = plot(robot.pos(1:3:(robot.nlink*3+1),end-28),robot.pos(2:3:(robot.nlink*3+3),end-28),'linewidth',10,'color',[1,0.9,0.9],'markersize',50);
        a1 = plot(agent.x(1,end-1),agent.x(2,end-1),'.','linewidth',3,'color',[0.50,0.50,1],'markersize',200);
        a2 = plot(agent.x(1,end-2),agent.x(2,end-2),'.','linewidth',3,'color',[0.55,0.55,1],'markersize',200);
        a3 = plot(agent.x(1,end-4),agent.x(2,end-4),'.','linewidth',3,'color',[0.60,0.60,1],'markersize',200);
        a4 = plot(agent.x(1,end-8),agent.x(2,end-8),'.','linewidth',3,'color',[0.65,0.65,1],'markersize',200);
        a5 = plot(agent.x(1,end-12),agent.x(2,end-12),'.','linewidth',3,'color',[0.70,0.70,1],'markersize',200);
        a6 = plot(agent.x(1,end-16),agent.x(2,end-16),'.','linewidth',3,'color',[0.75,0.75,1],'markersize',200);
        a7 = plot(agent.x(1,end-20),agent.x(2,end-20),'.','linewidth',3,'color',[0.80,0.80,1],'markersize',200);
        a8 = plot(agent.x(1,end-24),agent.x(2,end-24),'.','linewidth',3,'color',[0.85,0.85,1],'markersize',200);
        a9 = plot(agent.x(1,end-28),agent.x(2,end-28),'.','linewidth',3,'color',[0.90,0.90,1],'markersize',200);
        saveas(gcf, string(t)+'graph.png');
        delete(p1); delete(p2); delete(p3); delete(p4); delete(p5);
        delete(p6); delete(p7); delete(p8); delete(p9);
        delete(a1); delete(a2); delete(a3); delete(a4); delete(a5);
        delete(a6); delete(a7); delete(a8); delete(a9);
    end
    pause(0.05); 

end
dt = toc(t0)
disp('End Condition')
disp('TIME FINISHED')
disp(t<1001)
disp('ROBOT NO MORE GOALS')
disp(num_goal_counter_robot <= num_goals)
disp('AGENT NO MORE GOALS')
disp(num_goal_counter_agent <= num_goals)
set(text1handle,'string','Test ended')
