%% ROBUST UVMS TASK-PRIORITY EXERCISE
% This script assembles the main blocks of the control architecture:
%
%   task definitions -> ActionManager/iCAT -> kinematic simulator -> Unity
%
% At every iteration, the Kinematic Control Layer computes the system velocity ydot = [q_dot; v_nu].
% UvmsSim integrates this reference directly; in a real robot, a Dynamic Control Layer would track the same reference.
%
% Run this script with the current MATLAB folder set to the template root so that the relative paths below resolve consistently.

%% Make exercise dependencies visible to MATLAB
addpath('./tools');
addpath('./icat');
addpath('./robust_robot');
addpath('./tpik');
clc; clear; close all;

%% Configure and initialize the simulation
% dt is both the integration step and the nominal controller sample time.
dt       = 0.005;
endTime  = 20;

% UvmsModel owns state and kinematics; UvmsSim updates that shared handle.
robotModel = UvmsModel();
sim = UvmsSim(dt, robotModel, endTime);

% Unity is used only as viewer/sensor endpoint; it is not the simulator.
%unity = UnityInterface("192.168.64.2");
unity = UnityInterface("127.0.0.1");

%% Define one action as a strict-priority stack of tasks
% The first cell is the highest-priority task. Add safety tasks before the action objective when extending this exercise (for example, joint limits).
task_min_alt = TaskAltitude(3 , "disequality");
task_hor_att = TaskHorizontalAttitude();
task_veh_pos_cont = TaskVehicleTotal();

task_land = TaskAltitude(0.1,"equality");
task_veh_pos = TaskVehiclePosition();


safe_navigation = {task_min_alt,task_hor_att,task_veh_pos_cont};
landing = {task_land,task_hor_att,task_veh_pos};
unifiedList = {task_min_alt,task_land,task_hor_att,task_veh_pos_cont,task_veh_pos};

actionManager = ActionManager_es4(robotModel);
actionManager.addAction("safe_navigation", safe_navigation);
actionManager.addAction("landing", landing);
actionManager.addAction("unifiedList", unifiedList);
actionManager.setCurrentAction("safe_navigation");

task_set = safe_navigation;

%% Set desired poses
% Positions are in metres and orientations are [roll pitch yaw] in radians. Both poses are expressed in the inertial/world frame.
w_arm_goal_position = [12.2025, 37.3748, -39.8860]';
w_arm_goal_orientation = [0, pi, pi/2];
w_vehicle_goal_position = [12.2025 37.3748 -39.8860]';
w_vehicle_goal_orientation = [0, 0, 0];

% Store the goals as homogeneous transforms in the shared model.
robotModel.setGoal(w_arm_goal_position, w_arm_goal_orientation, w_vehicle_goal_position, w_vehicle_goal_orientation);

% Preallocate one column for the initial state plus all integration steps.
logger = SimulationLogger(ceil(endTime/dt)+1, robotModel, task_set);

%% Run the closed-loop kinematic simulation
tic
for step = 1:sim.maxSteps
    % Read the newest sensor sample. If none is available, the interface keeps the previous altitude value.
    robotModel.altitude = unity.receiveAltitude(robotModel);

    % Update every task and solve the current strict-priority hierarchy.
    [v_nu, q_dot] = actionManager.computeICAT(robotModel);

    % Integrate arm and vehicle velocities, then refresh all transforms.
    sim.step(v_nu, q_dot);

    % Send the state after integration so the viewer displays the new sample.
    unity.send(robotModel);

    % Store state, task references and activations at the current sample.
    logger.update(sim.time, sim.loopCounter);

    % Print a compact status update approximately once per simulated second.
    if mod(sim.loopCounter, round(1 / sim.dt)) == 0
        fprintf('t = %.2f s\n', sim.time);
        fprintf('alt = %.2f m\n', robotModel.altitude);
    end

    if norm(robotModel.eta(1:3) - w_vehicle_goal_position) < 0.05
         
        actionManager.setCurrentAction("lander");
       
    end

    % Busy-wait until one sample period has elapsed when real-time playback is desired. 
    % Remove this call to execute the simulation as fast as possible.
    SlowdownToRealtime(dt);
end

%% Review results and release communication resources
logger.plotAll();

% Explicit deletion closes UDP sockets before the script terminates.
delete(unity);
