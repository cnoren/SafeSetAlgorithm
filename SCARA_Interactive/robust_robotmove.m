%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  ROBOT CONTROL CODE              %
%  CHARLES NOREN                   %
%  2021.2                          %
%                                  %
%  ORIGINALLY BY:                  %
%  CHANGLIU LIU                    %
%  2015.5                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [status,robotnew]=robust_robotmove(t,robot)

status=0;

if status==0
    goal=[robot.Goal(1,1);robot.Goal(2,1)];
    robot.goalhis{t}=goal;
    
    % CALCULATE THE JACOBIAN
    [Je,H, ~, ~, He]=Jacobi(robot.x(1:robot.nlink,end),robot.x(robot.nlink+1:2*robot.nlink,end),robot.l,robot.nlink,robot.l(end),robot.base,robot.DH);
    % STEPWISE CALCULATION OF THE DYNAMIC MATRICES
    [M,C]=rbt_dyna_matrix(robot.x(:,end),robot.param);
    [M_hat,~]=rbt_dyna_matrix(robot.x(:,end),robot.est_param);
    C_hat = calc_chat(robot.x(robot.nlink+1:2*robot.nlink,end), robot.x(robot.nlink+1:2*robot.nlink,end), robot.est_param);
    % DESIRED CONTROL
    % This is the STANDARD PD CONTROL ON THE ROBOT AS OF NOW in TASK SPACE
    % U=-Je'*(robot.wx(1:2,end)-goal+1*robot.wx(4:5,end))*100-robot.x(robot.nlink+1:2*robot.nlink,end);
    [qrdot, qrddot, s] = calc_sliding_vars(robot.x(robot.nlink+1:2*robot.nlink,end), goal, robot.wx(1:2,end), robot.wx(4:5,end), Je, He, robot.slotinecontroller.LAMBDA);    
    U = compute_control_slotine(qrddot, qrdot, s, M_hat, C_hat, robot.slotinecontroller.KD);
    robot.u(1:2,t)=U; % STORE THIS FOR USE IN THE SYSTEM
    robot.est_param = est_params(robot.x(robot.nlink+1:2*robot.nlink,end), robot.x(robot.nlink+1:2*robot.nlink,end), qrdot, qrddot, robot.delta_t, robot.slotinecontroller.Gamma, s, robot.est_parambarrier,  robot.est_param);
    disp(robot.est_param)
    [~, uwidth] = size(U);
    if uwidth == 2
        disp('yikes1')
    end
    tic % FOR CODE TIMING PURPOSES CHECKS
    
    % CALCULATE THE CLOSEST POINT ON THE ROBOT TO THE ACTOR/AGENT/CURSOR
    [linkid,ratio,robot.profile{t}]=closest_2D(robot.obs.xstar(1:2,t),robot.x(:,t),robot.pos(:,t),robot.l);
    robot.profile{t}.dtime=toc; % COMPUTE HOW LONG IT TAKES TO CALCULATE THE 2D 
    tic;
    robot.profile{t}.ssa=0;
    if linkid==1 && ratio==0
        if robot.profile{t}.rmin<0.1
            ratio=0.001;
        end
    end
    
    if linkid==1 && ratio==0
        robot.u(1:2,t)=U;
    else
        [Jm,Hm,robot.mx(1:3,t),robot.mx(4:6,t)]=Jacobi(robot.x(1:robot.nlink,end),robot.x(robot.nlink+1:2*robot.nlink,end),robot.l,linkid,ratio,robot.base,robot.DH);
        [alpha_cond, M_safe, M_safe2, flag] = evolutionary(robot.centerpoint, robot.lowerpoint, robot.upperpoint, Jm, robot.obs.xstar(1:2,t), robot.x(1:2,t), robot.range, robot.grids);
        M_comb = alpha_cond*M_safe;
        %M_comb = (M_safe + M_safe2)/2;
        if flag
            plotLgPhis(M, M_hat, alpha_cond*M_safe, alpha_cond*M_safe2, alpha_cond*M_comb, Jm, robot.x(1:2,t))
        end
        M_hat = M_comb;
        %[alpha_condLP, M_safeLP, M_safeLP2] = evolutionary(robot.lowerpoint, robot.lowerpoint, robot.upperpoint, Jm, robot.obs.xstar(1:2,t), robot.x(1:2,t), robot.range);
        %[alpha_condUP, M_safeUP, M_safeUP2] = evolutionary(robot.upperpoint, robot.lowerpoint, robot.upperpoint, Jm, robot.obs.xstar(1:2,t), robot.x(1:2,t), robot.range);
        % Discrete safety index
        BJ=robot.B*Jm*pinv(M_hat);
        %D=(robot.A-robot.inf.B{t}(:,1:4))*robot.wx([1,2,4,5],end)-robot.inf.A{t}*robot.obs.xstar(:,end)-robot.inf.B{t}(:,5:6)*robot.obs.goal(:,end)+robot.B*Hm;
        
        D=robot.A*robot.mx([1,2,4,5],end)+robot.B*(Hm-Jm*pinv(M_hat)*C_hat*robot.x(robot.nlink+1:2*robot.nlink,end))-robot.obs.xstar(:,end);
        
        [thres,vet]=safety(D,BJ,robot.margin);
        
        
        
        if (vet*U)<thres
            change=thres-vet*U;
            U=U+M_hat*vet'*pinv(vet*M_hat*vet')*change; % PROBLEM!
            robot.profile{t}.ssa=1;
        end
        
        dx=robot.mx([1,2,4,5],end)-robot.obs.xstar(:,end);
        dmin=robot.profile{t}.rmin;
        kd=0.01;
        % Continuous safety index
        if 0.15^2-dmin^2-kd*dx'*robot.const.P2*dx/dmin>=0
            vet=dx(1:2)'*Jm*pinv(M_hat)./dmin;
            vcirc=dx(3:4)-dx(1:2)*(dx(3:4)'*dx(1:2)/dmin);
            thres=robot.margin-2*dx'*robot.const.P2*dx+kd*(dx(1:2)'*(Jm*pinv(M_hat)*C_hat*robot.x(robot.nlink+1:2*robot.nlink,end)-Hm)-norm(vcirc))/dmin;
            if (vet*U)<thres
                change=thres-vet*U;
                U=U+M_hat*vet'*pinv(vet*M_hat*vet')*change;
                robot.profile{t}.ssa=1;
            end
        end
    end
    if uwidth == 2
        disp('yikes2')
    end
    robot.profile{t}.ssatime=toc;
    
    
    if abs(U(1))>robot.umax(1)
        U(1)=U(1)/abs(U(1))*robot.umax(1);
    end
    if abs(U(2))>robot.umax(2)
        U(2)=U(2)/abs(U(2))*robot.umax(2);
    end
    
    if robot.x(3,end)*robot.u(1,t)/norm(robot.u(1,t))>=robot.wmax
        robot.u(1,t)=0;
    end
    if robot.x(4,end)*robot.u(2,t)/norm(robot.u(2,t))>=robot.wmax
        robot.u(2,t)=0;
    end
    
    [~, uwidth] = size(U);
    if uwidth == 2
        disp('yikes3')
    end
    robot.u(1:2,t)=U;
    
    
    
end

[~,Q]=ode45(@(t,x)rbt_fwd_dyna_frc(t,x,robot.param,robot.u(:,end)), [t*robot.delta_t,(t+1)*robot.delta_t],robot.x(:,end));

robot.x(:,t+1)=Q(end,:);
robot.pos(:,t+1)=ArmPos(robot.base,robot.DH,robot.x(1:robot.nlink,t+1));%(x1,y1,z1,...,x(N+1),y(N+1),z(N+1))
robot.wx(1:3,t+1)=robot.pos(end-2:end,t+1);
robot.wx(4:5,t+1)=Je*robot.x(robot.nlink+1:2*robot.nlink,t+1);

robotnew=robot;
end