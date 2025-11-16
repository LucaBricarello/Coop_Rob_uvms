% Add paths
addpath('./simulation_scripts');
addpath('./tools');
addpath('./icat');
addpath('./robust_robot');
addpath('include');
clc; clear; close all;

% Simulation parameters
dt       = 0.05;
endTime  = 50;
% Initialize robot model and simulator
robotModel = UvmsModel();          
sim = UvmsSim(dt, robotModel, endTime);
% Initialize Unity interface
unity = UnityInterface("127.0.0.1");

% Define tasks
%task_vehicle = TaskVehicle();       
%task_tool    = TaskTool();
%task_position    = TaskPosition();
task_horizontal    = TaskHorizontal();
%task_min_altitude    = TaskMinAltitudeV2();
task_set = {sdtask_horizontal};
%task_set_2 = {task_land};
%unified_task_list = {task_min_altitude, task_land, task_position, task_horizontal};

% Define actions and add to ActionManager
actionManager = ActionManager();
actionManager.addAction(task_set, "safe navigation");  % action 1
%actionManager.addAction(task_set_2, "landing");  % action 2
%actionManager.addUnifiedList(unified_task_list);

% Define desired positions and orientations (world frame)
w_arm_goal_position = [12.2025, 37.3748, -39.8860]';
w_arm_goal_orientation = [0, pi, pi/2];
w_vehicle_goal_position = [10.5, 37.5, -39.9]';
w_vehicle_goal_orientation = [0, 0, 0];

% Set goals in the robot model
robotModel.setGoal(w_arm_goal_position, w_arm_goal_orientation, w_vehicle_goal_position, w_vehicle_goal_orientation);

% Initialize the logger
logger = SimulationLogger(ceil(endTime/dt)+1, robotModel, task_set);

% Main simulation loop
for step = 1:sim.maxSteps
    
    % print position on screen
    disp(robotModel.eta)

    % 1. Receive altitude from Unity
    robotModel.altitude = unity.receiveAltitude(robotModel);

    % 2. Compute control commands for current action
    [v_nu, q_dot] = actionManager.computeICAT(robotModel);

    % 3. Step the simulator (integrate velocities)
    sim.step(v_nu, q_dot);

    % 4. Send updated state to Unity
    unity.send(robotModel);

    % 5. Logging
    logger.update(sim.time, sim.loopCounter);

    % 6. Optional debug prints
    if mod(sim.loopCounter, round(1 / sim.dt)) == 0
        fprintf('t = %.2f s\n', sim.time);
        fprintf('alt = %.2f m\n', robotModel.altitude);
    end

    % 7. Optional real-time slowdown
    SlowdownToRealtime(dt);
end

% Display plots
logger.plotAll();

% Clean up Unity interface
delete(unity);