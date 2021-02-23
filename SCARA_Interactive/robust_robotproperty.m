function robot=robust_robotproperty(id)
switch id
    case 1
        %the constants
        robot.nlink=2;
        robot.umax=[5,2]; %m/s^2
        robot.margin=0.05;
        robot.delta_t=0.05;
        robot.wmax=1; %rad/s
        robot.seed = rng(0, 'twister');
        %The moments, lengths of the links and DH parameter and base
        
        % With the Robust Safe Set Algorithm, we need a range for each fo
        % the parameters
        
        % ORIGINAL SCARA PARAMETERS:
%         robot.range.IL1 = [0.123 0.123]; % Range of Inertia of Link 1 (L1)
%         robot.range.IL2 = [0.028 0.028]; % Range of Inertia of Link 2 (L2)
%         robot.range.lL1 = [0.320 0.320]; % Range of L1 Length
%         robot.range.lL2 = [0.215 0.215]; % Range of L2 Length
%         robot.range.mL1 = [6.830 6.830]; % Range of Mass of L1
%         robot.range.mL2 = [3.290 3.290]; % Range of Mass of L2
%         robot.range.mM2 = [5.560 5.560]; % Range of Mass of the 2nd motor
%         robot.range.mEE = [1.050 1.050]; % Range of Mass of the end effector
%         robot.range.TC1 = [22.36 22.36]; % Range of Motor 1 Torque Constant
%         robot.range.TC2 = [2.420 2.420]; % Range of Motor 2 Torque Constant

        % RANGE OF PARAMETERS WE ARE USING:
        robot.range.IL1 = [00.10 00.20]; % Range of Inertia of Link 1 (L1)
        robot.range.IL2 = [00.01 00.03]; % Range of Inertia of Link 2 (L2)
        robot.range.lL1 = [00.30 00.40]; % Range of L1 Length
        robot.range.lL2 = [00.21 00.22]; % Range of L2 Length
        robot.range.mL1 = [06.80 06.85]; % Range of Mass of L1
        robot.range.mL2 = [03.25 03.30]; % Range of Mass of L2
        robot.range.mM2 = [05.50 05.60]; % Range of Mass of the 2nd motor
        robot.range.mEE = [01.04 01.06]; % Range of Mass of the end effector
        robot.range.TC1 = [22.35 22.37]; % Range of Motor 1 Torque Constant
        robot.range.TC2 = [02.40 02.45]; % Range of Motor 2 Torque Constant
        % CALCULATE THE RANGE:
        IL1 = computeValInRange(robot.range.IL1(1), robot.range.IL1(2), robot.seed);
        IL2 = computeValInRange(robot.range.IL2(1), robot.range.IL2(2), robot.seed);
        Il1 = computeValInRange(robot.range.lL1(1), robot.range.lL1(2), robot.seed);
        Il2 = computeValInRange(robot.range.lL2(1), robot.range.lL2(2), robot.seed);
        mL1 = computeValInRange(robot.range.mL1(1), robot.range.mL1(2), robot.seed);
        mL2 = computeValInRange(robot.range.mL2(1), robot.range.mL2(2), robot.seed);
        mM2 = computeValInRange(robot.range.mM2(1), robot.range.mM2(2), robot.seed);
        mEE = computeValInRange(robot.range.mEE(1), robot.range.mEE(2), robot.seed);
        TC1 = computeValInRange(robot.range.TC1(1), robot.range.TC1(2), robot.seed);
        TC2 = computeValInRange(robot.range.TC2(1), robot.range.TC2(2), robot.seed);
        
        robot.I    =[IL1 IL2]; %Moments of inertia (kg.m^2)
        robot.l    =[Il1 Il2]; %Length of the links (m)
        robot.m    =[mL1 mL2]; %Mass of the links (Kg)
        robot.M    =[mM2 mEE]; %Mass of the second motor and the end-effector
        robot.Kt   =[TC1 TC2]; %Torque constant of the motors (N.m/V)
        robot.DH   =[0 0 robot.l(1) 0;pi/4 0 robot.l(2) 0]; %theta,d,a,alpha
        robot.base =[-30;-30;0]./100; %origin
        
        %Parameters in dynamics
        robot.param=[];
        % There is an implicit assumption here that the center of mass of
        % each of the links is located at the length-centroid of the link
        % We don't use the extra masses that are currently written out
        robot.param(1)=robot.I(1)+robot.I(2)+(robot.m(1)/4+robot.m(2))*robot.l(1)^2+...
            robot.m(2)*robot.l(2)^2/4;
        robot.param(2)=robot.I(2)+robot.m(2)*robot.l(2)^2/4;
        robot.param(3)=robot.m(2)*robot.l(1)*robot.l(2)/2;

end


%The kinematic matrices
robot.A=[eye(robot.nlink) robot.delta_t*eye(robot.nlink);zeros(robot.nlink) eye(robot.nlink)];
robot.B=[0.5*robot.delta_t^2*eye(robot.nlink);robot.delta_t*eye(robot.nlink)];
robot.C=eye(2*robot.nlink);
robot.D=zeros(2*robot.nlink,robot.nlink);
robot.Q=diag([ones(1,robot.nlink) zeros(1,robot.nlink)]);%[1 robot.delta_t 0 0;robot.delta_t robot.delta_t^2 0 0;0 0 1 robot.delta_t;0 0 robot.delta_t robot.delta_t^2];
robot.R=eye(robot.nlink);
robot.Goal=[-0.2,-0.1]';
robot.nG=size(robot.Goal,2);

robot.x(1:2*robot.nlink,1)=[robot.DH(:,1);zeros(robot.nlink,1)];%(theta1,...,thetaN,theta1dot,...,thetaNdot)
robot.pos=ArmPos(robot.base,robot.DH,robot.x(1:robot.nlink,1));%(x1,y1,z1,...,x(N+1),y(N+1),z(N+1))
robot.wx(1:3*robot.nlink,1)=[robot.pos(end-2:end);0;0;0];%endpoint state(x(N+1),y(N+1),z(N+1),x(N+1)dot,y(N+1)dot,z(N+1)dot)
robot.mx=robot.wx;%closest point state

robot.ref.x=robot.x;
robot.innoise=0;
robot.outnoiseself=0;
robot.outnoisestar=0;
robot.obs.xself=[];
robot.obs.xstar=[];
robot.obs.goal=[];
robot.obs.A=robot.A;
robot.obs.B=robot.B;
robot.obs.C=robot.C;
robot.obs.D=robot.D;
robot.obs.Q=robot.Q;
robot.obs.R=robot.R;
robot.score=0;
robot.inf.A={};
robot.inf.B={};
robot.inf.F={};
robot.inf.A{1}=robot.A;
robot.inf.B{1}=[eye(4) robot.B];
robot.inf.F{1}=eye(10);
robot.flag=0;

%For SSA
robot.const.P1=[eye(robot.nlink) zeros(robot.nlink);zeros(robot.nlink) zeros(robot.nlink)];
robot.const.P2=[zeros(robot.nlink) 0.5*eye(robot.nlink);0.5*eye(robot.nlink) zeros(robot.nlink)];
robot.const.P3=[zeros(robot.nlink) zeros(robot.nlink);zeros(robot.nlink) eye(robot.nlink)];

%For collision check
robot.profile={};





