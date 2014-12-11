options = struct();
%options.floating = true;
%options.replace_cylinders_with_capsules = false;
%options.use_bullet = true;
plant = RigidBodyManipulator('PushArm.urdf', options);
% todo figure out how to set this properly
x0 = zeros(8,1);
xf_min = -9 * ones(8,1);
xf_max =  9 * ones(8,1);
xf_min(1) = 0.4;
xf_max(1) = 0.6;

%x0(1) = 1;
%x0(2) = 1;
%x0(3) = 1;

N=5; tf=.5;
plant_ts = TimeSteppingRigidBodyManipulator(plant,tf/(N-1));
%w = warning('off','Drake:TimeSteppingRigidBodyManipulator:ResolvingLCP');
xtraj_ts = simulate(plant_ts,[0 tf],x0);
x0 = xtraj_ts.eval(0);
%warning(w);
visualize = true;
if visualize
    v = constructVisualizer(plant_ts);
    v.playback(xtraj_ts);
end

options = struct();
options.integration_method = ContactImplicitTrajectoryOptimization.MIXED;
scale_sequence = [1;.001;0];
for i=1:length(scale_sequence)
    scale = scale_sequence(i);
    options.compl_slack = scale*.01;
    options.lincompl_slack = scale*.001;
    options.jlcompl_slack = scale*.01;
    prog = ContactImplicitTrajectoryOptimization(plant,N,tf,options);
    prog = prog.setSolver('snopt');
    prog = prog.setSolverOptions('snopt','MajorIterationsLimit',200);
    prog = prog.setSolverOptions('snopt','MinorIterationsLimit',200000);
    prog = prog.setSolverOptions('snopt','IterationsLimit',200000);
    % prog = prog.setCheckGrad(true);
    % snprint('snopt.out');
    % initial conditions constraint
    prog = addStateConstraint(prog,ConstantConstraint(x0),1);
    prog = addStateConstraint(prog,BoundingBoxConstraint(xf_min,xf_max),N);
    
    if i == 1,
        traj_init.x = PPTrajectory(foh([0,tf],[x0,x0]));
    else
        traj_init.x = xtraj;
        traj_init.l = ltraj;
    end
    [xtraj,utraj,ltraj,~,z,F,info] = solveTraj(prog,tf,traj_init);
end
if visualize
    v.playback(xtraj);
end