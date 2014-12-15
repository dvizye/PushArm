%% Timestepping version that respects contacts
N = 10; % Number of knot points
T_span = 3; % Time of traj
plant_ts = TimeSteppingRigidBodyManipulator('PushArm.urdf',T_span/(N-1));
xtraj_ts = simulate(plant_ts,[0 T_span],x0);
if true
    v = constructVisualizer(plant_ts);
    v.playback(xtraj_ts);
end
%% Testing forward kinematics
options = struct();
x0 = zeros(8, 1);
p = RigidBodyManipulator('PushArm.urdf', options);
kinsol = p.doKinematics(x0(1:4, 1), false, true, x0(5:8, 1));
lowest_link = p.findLinkInd('ball', 0, 0);
[x, J, dJ] = p.forwardKin(kinsol, lowest_link, [0; 0; 0], 0)
%% Visualize
visualize = true;
if visualize
    v = p.constructVisualizer();
    v.inspector();
end