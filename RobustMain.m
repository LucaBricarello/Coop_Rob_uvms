% Luca Bricarello 5248168
% Barbara Ratto 4952202
% Negri Ravera Christian 5228863

% Add paths
addpath('./simulation_scripts');
addpath('./tools');
addpath('./icat');
addpath('./robust_robot');
addpath('include');
addpath('./tasks')
clc; clear; close all;

% Simulation parameters
dt       = 0.05;
endTime  = 90;
% Initialize robot model and simulator
robotModel = UvmsModel();          
sim = UvmsSim(dt, robotModel, endTime);
% Initialize Unity interface
unity = UnityInterface("127.0.0.1");

% Define tasks
task_position        = TaskPosition();
task_horizontal      = TaskHorizontal();
task_min_altitude    = TaskMinAltitude();
task_land            = TaskLand(0);
task_orientation     = TaskOrientation();
task_stop_move       = TaskStopMove();
task_allign_to_obj   = TaskAllignToObj();
task_dist_v_to_obj   = TaskDistVehicleToNodule();
task_tool            = TaskTool();

move_to_point = {task_min_altitude, task_horizontal, task_position, task_orientation};
pre_land = {task_min_altitude, task_horizontal, task_dist_v_to_obj, task_allign_to_obj};
land = {task_horizontal, task_dist_v_to_obj, task_allign_to_obj, task_land};
manipulation = {task_stop_move, task_tool};

unified_task_list = {task_stop_move, task_min_altitude, task_horizontal, task_dist_v_to_obj, task_allign_to_obj, task_land, task_position, task_orientation, task_tool};

% Define actions and add to ActionManager
actionManager = ActionManager();

actionManager.addAction(move_to_point, "safe_navigation");
actionManager.addAction(pre_land, "pre_landing");
actionManager.addAction(land, "safe_landing");
actionManager.addAction(manipulation, "manipulation");

actionManager.addUnifiedList(unified_task_list);

% Define desired positions and orientations (world frame)
w_arm_goal_position = [12.2025, 37.3748, -39.8860]'; % <--- NODULE FRAME (is stored in wTg)
w_arm_goal_orientation = [0, pi, pi/2];
w_vehicle_goal_position = [10.5 38.5 -38]'; % (is stored in wTgv)
%w_vehicle_goal_position = [10.5 38.5 -39.8860]'; % Goal per min alt test
w_vehicle_goal_position = [6.5 40.5 -38]'; % Goal dist v to obj test
w_vehicle_goal_orientation = [0, -0.06, 0.5];

% Set goals insss the robot model
robotModel.setGoal(w_arm_goal_position, w_arm_goal_orientation, w_vehicle_goal_position, w_vehicle_goal_orientation);

% Initialize the logger
logger = SimulationLogger(ceil(endTime/dt)+1, robotModel, unified_task_list, actionManager);

switch_flag_1 = 0;
switch_flag_2 = 0;
switch_flag_3 = 0;

t_switch_1 = 0;
t_switch_2 = 0;
t_switch_3 = 0;

% Main simulation loop
for step = 1:sim.maxSteps

    % --------- Mission planning part -------------
    if strcmp(actionManager.actions_name{actionManager.currentAction}, "safe_navigation")
        disp("action 1")
        if (task_position.error < 0.01) & (switch_flag_1 == 0) % (task_orientation.error < 0.01) & (switch_flag_1 == 0)
            actionManager.setCurrentAction("pre_landing");
            switch_flag_1 = 1;
            t_switch_1 = sim.time;
        end
    end
    if strcmp(actionManager.actions_name{actionManager.currentAction}, "pre_landing")
        disp("action 2")
        if (task_allign_to_obj.error < 0.01) & (switch_flag_2 == 0)
            actionManager.setCurrentAction("safe_landing");
            switch_flag_2 = 1;
            t_switch_2 = sim.time;
        end
    end
    if strcmp(actionManager.actions_name{actionManager.currentAction}, "safe_landing")
        disp("action 3")
        if (task_land.error < 0.05) & (switch_flag_3 == 0)
            actionManager.setCurrentAction("manipulation");
            switch_flag_3 = 1;
            t_switch_3 = sim.time;
        end
    end
    if strcmp(actionManager.actions_name{actionManager.currentAction}, "manipulation")
        disp("action 4")
    end
    % ---------------------------------------------
    
    % print position on screen
    disp(robotModel.eta)

    % 1. Receive altitude from Unity
    robotModel.altitude = unity.receiveAltitude(robotModel);

    % 2. Compute control commands for current action
    [v_nu, q_dot] = actionManager.computeICAT(robotModel, dt);

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

logger.t_switch_1 = t_switch_1;
logger.t_switch_2 = t_switch_2;
logger.t_switch_3 = t_switch_3;


% Display plots
logger.plotAll();

% Clean up Unity interface
delete(unity);