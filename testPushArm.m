function [p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt] = testPushArm(xtraj,utraj,ltraj,ljltraj,scale)
    options = struct;
%     options.ignore_self_collisions = false;
    p = RigidBodyManipulator('PushArm.urdf', options);


    % Goals
    x0 = zeros(8, 1);
    x0(2) = -pi/3;
%     xf = zeros(8, 1);
%     xf(2) = -pi/3;
%     xf = [2.77; 0.698; -0.614; 1.222; 0; 0; 0; 0];
    xf = [    0
              0.7600
              -0.9800
              1.4540
                0
                0
                0
                0];
    xf_min = -inf(8, 1);
    xf_min(1) = 2.5;
    xf_max = inf(8, 1);
    xf_max(1) = 2.5;
    % Configure traj_opt
    N = 3; % Number of knot points
    T_span = 3; % Time of traj
    T0 = 3;
    t_init = linspace(0, T0, N);
    traj_init.x = PPTrajectory(foh(t_init,[linspacevec(x0,xf,N)]));
    traj_init.u = PPTrajectory(foh(t_init,randn(3,N)));
    w = warning('off','Drake:RigidBodyManipulator:ReplacedCylinder');
    
    warning(w); 

    to_options.nlcc_mode = 2;
    to_options.lincc_mode = 1;
    to_options.compl_slack = .01;
    to_options.lincompl_slack = .001;
    to_options.jlcompl_slack = .01;
    to_options.lambda_mult = p.getMass*9.81*T0/N;
    to_options.lambda_jl_mult = T0/N;

    traj_opt = ArmContactTrajectoryOptimization(p,N,T_span,to_options);
    traj_opt = traj_opt.addStateConstraint(ConstantConstraint(x0), 1);
%     traj_opt = traj_opt.addStateConstraint(ConstantConstraint(xf), N);
    traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(xf_min,xf_max),N);
    traj_opt = traj_opt.addRunningCost(@running_cost_fun);
    snprint('snopt.out');
    traj_opt = traj_opt.setSolver('snopt');
    traj_opt = traj_opt.setSolverOptions('snopt','derivativeoption', 0);
    traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',200);
    traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',200000);
    traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',200000);
    traj_opt = traj_opt.setCheckGrad(true);
    [xtraj,utraj,ltraj,ljltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);
    v = p.constructVisualizer;
    v.playback(xtraj);

    function [f,df] = running_cost_fun(h,x,u)
      f = u'*u;
      % [df/dt df/dx df/du]
      df = [0 zeros(1,8) 2*u'];
    end
end

