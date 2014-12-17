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
    x1 = [0; -0.736; 0.973; 1.091; 0; 0; 0; 0]; 
%     xf = [    0
%               0.7600
%               -0.9800
%               1.4540
%                 0
%                 0
%                 0
%                 0];
    xf = [2.414; 0.137; 0.321; 1.099; 0; 0; 0; 0];
    xf_min = -inf(8, 1);
    xf_min(1) = 2.3;
    xf_max = inf(8, 1);
    xf_max(1) = 2.5;
    % Configure traj_opt
    N = 3; % Number of knot points
    N2 = floor(N/2);
    T_span = 3; % Time of traj
    T0 = 3;
    w = warning('off','Drake:RigidBodyManipulator:ReplacedCylinder');
    if nargin < 2
      %Try to come up with a reasonable trajectory
      scale = 0.1;
      t_init = linspace(0,T0,N);
%       traj_init.x = PPTrajectory(foh(t_init,linspacevec(x0,xf,N)));
      traj_init.x = PPTrajectory(foh(t_init,[linspacevec(x0,x1,N2), linspacevec(x1,xf,N-N2)]));
      traj_init.u = PPTrajectory(foh(t_init,randn(3,N)));
    else
      t_init = xtraj.pp.breaks;
      traj_init.x = xtraj;
      traj_init.u = utraj;
      traj_init.l = ltraj;
      traj_init.ljl = ljltraj;
    end
    
    warning(w); 

    to_options.compl_slack = scale*.01;
    to_options.lincompl_slack = scale*.001;
    to_options.jlcompl_slack = scale*.01;
    to_options.nlcc_mode = 2;
    to_options.lincc_mode = 1;
%     to_options.lambda_mult = p.getMass*9.81*T0/N;
%     to_options.lambda_jl_mult = T0/N;
    to_options.lambda_mult = 1;
    to_options.lambda_jl_mult = 1;

    traj_opt = ArmContactTrajectoryOptimization(p,N,T_span,to_options);
    traj_opt = traj_opt.addStateConstraint(ConstantConstraint(x0), 1);
%     traj_opt = traj_opt.addStateConstraint(ConstantConstraint(xf), N);
%     traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(xf_min,xf_max),N);
    traj_opt = traj_opt.addRunningCost(@running_cost_fun);
    snprint('snopt.out');
    traj_opt = traj_opt.setSolver('snopt');
    traj_opt = traj_opt.setSolverOptions('snopt','derivativeoption', 0);
    traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',200);
    traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',200000);
    traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',200000);
%     traj_opt = traj_opt.setCheckGrad(true);
    [xtraj,utraj,ltraj,ljltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);
    v = p.constructVisualizer;
    v.playback(xtraj);

    function [f,df] = running_cost_fun(h,x,u)
      f = u'*u;
      % [df/dt df/dx df/du]
      df = [0 zeros(1,8) 2*u'];
    end
end

