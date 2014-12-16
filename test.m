% %% Timestepping version that respects contacts
% N = 10; % Number of knot points
% T_span = 3; % Time of traj
% plant_ts = TimeSteppingRigidBodyManipulator('PushArm.urdf',T_span/(N-1));
% xtraj_ts = simulate(plant_ts,[0 T_span],x0);
% if true
%     v = constructVisualizer(plant_ts);
%     v.playback(xtraj_ts);
% end
% %% Testing forward kinematics
options = struct();
x0 = zeros(8, 1);
p = RigidBodyManipulator('PushArm.urdf', options);
kinsol = p.doKinematics(x0(1:4, 1), false, true, x0(5:8, 1));
lowest_link = p.findLinkInd('link_1', 0, 0);
[x, J, dJ] = p.forwardKin(kinsol, lowest_link, [0; 0; 0], 0)
%% Visualize
visualize = true;
options = struct();
p = RigidBodyManipulator('PushArm.urdf', options);
if visualize
    v = p.constructVisualizer();
%     x0 = [1.0654; 0.0867; 0.0073; -5.0561];
%     x0 = [1.0654; 0.0867; 0.0073; -5.0561; 0; 0; 0; 0]
    v.inspector();
end
%% Check distance
% Input xtraj (i.e. run this after running testPushArm)
% Output distance between first coordinate and 4 coordinate
% clear all;
[p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt] = testPushArm
N = 10;
phi_knot_points = zeros(1, N);
phi_ind = 1;
x_knot_points = zeros(8, N);
for n = 1:N
    disp(n)
    t_interval = 3 / (N-1);
    t = (n-1) * t_interval;
    x_i = xtraj.eval(t);
    kinsol = p.doKinematics(x_i(1:4, 1), false, true, x_i(5:8, 1));
    options.active_collision_options.terrain_only = false;
    [phi,normal,d,xA,xB,idxA,idxB,mu,n,D,dn,dD] = p.contactConstraints(kinsol,false,options.active_collision_options);
    for j = 1:4
        disp(j);
        z = D{j};
        disp(norm(z));
    end
    phi_knot_points(phi_ind) = phi;
    x_knot_points(:, phi_ind) = x_i;
    phi_ind = phi_ind + 1;
    
    ball = p.findLinkInd('ball', 0, 0);
    lowest_link = p.findLinkInd('base_link', 0, 0);
    [link_x, ~, ~] = p.forwardKin(kinsol, lowest_link, [0; 0; 0]);
end
%% Visualize xtraj
v = p.constructVisualizer();
options.slider = true;
v.playback(xtraj, options);