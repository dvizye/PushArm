function [p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt] = PushArm(xtraj,utraj,ltraj,ljltraj,scale)
    options.floating = true;
    options.terrain = RigidBodyFlatTerrain;
    p = RigidBodyManipulator('PushArm.urdf');
    p.getNumInputs()
    % x0 = zeros(
    T0 = 3;
    T = 3;
    N = 15;
    t_init = linspace(0, T0, 15);
    T_span = [1 T];
    scale = 0.1;
    to_options.compl_slack = scale*.01;
    to_options.lincompl_slack = scale*.001;
    to_options.jlcompl_slack = scale*.01;

    to_options.nlcc_mode = 2;
    to_options.lincc_mode = 1;
    to_options.lambda_mult = p.getMass*9.81*T0/N;
    to_options.lambda_jl_mult = T0/N;

    traj_opt = ContactImplicitTrajectoryOptimization(p,N,T_span,to_options);
    traj_opt.addRunningCost(@running_cost_fun);
    traj_opt.addFinalCost(@final_cost_fun);

    % Write function translating joint angles into z's, and write
    % constraint on z
    % function [z] = forward_kinematics(q)
    % cnstr = FunctionHandleConstraint(zeros(nX,1),zeros(nX,1),n_vars,@obj.dynamics_constraint_fun);
    %function [z] = forward_kinematics(q)
    %    z1 = 

    % Add constraints on initial and final conditions
    x0 = zeros(8, 1);
    x0_min = x0;
    x0_max = x0;
    xf_min = x0;
    xf_max = x0;
    
    traj_init.x = PPTrajectory(foh(t_init,[linspacevec(x0,x1,N2), linspacevec(x1,xf,N-N2)]));
    traj_init.u = PPTrajectory(foh(t_init,randn(3,N)));
    traj_init.l = PPTrajectory(foh(t_init,[repmat([1;zeros(7,1)],1,N2) repmat([zeros(4,1);1;zeros(3,1)],1,N-N2)]));
    traj_init.ljl = PPTrajectory(foh(t_init,zeros(p.getNumJointLimitConstraints,N)));
    
    traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(x0_min,x0_max),1);
    traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(xf_min,xf_max),N);
    traj_opt = traj_opt.setSolver('snopt');
    traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',200);
    traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',200000);
    traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',200000);
    [xtraj,utraj,ltraj,ljltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);
    % Add complementary constraint from pushing ball with object

    function [f,df] = running_cost_fun(h,x,u)
      f = u'*u;
      df = [0 zeros(1,12) 2*u'];
    end

    function [f,df] = final_cost_fun(T,x)
      K = 100;
      f = K*T;
      df = [K zeros(1,12)];
    end
end