classdef ActionManager < handle
    properties
        actions = {}      % cell array of actions (each action = stack of tasks)
        actions_name = {} % array containing names of the actions (at the same index)
        unified_list = {}
        currentAction = 1 % index of currently active action
        previousAction = 1
        time = 0
        a_curr = 1
        a_prev = 0

    end

    methods
        function addAction(obj, taskStack, name_of_action)
            % taskStack: cell array of tasks that define an action
            obj.actions{end+1} = taskStack;
            obj.actions_name{end+1} = name_of_action;
        end

        function addUnifiedList(obj, unified_list_input)
            obj.unified_list = unified_list_input;
        end

       function [v_nu, qdot] = computeICAT2(obj, robot)
           % Get current action
           tasks = obj.actions{obj.currentAction};

           % 1. Update references, Jacobians, activations
           for i = 1:length(tasks)
               tasks{i}.updateReference(robot);
               tasks{i}.updateJacobian(robot);
               tasks{i}.updateActivation(robot);
           end

           % 2. Perform ICAT (task-priority inverse kinematics)
           ydotbar = zeros(13,1);
           Qp = eye(13);
           for i = 1:length(tasks)
               [Qp, ydotbar] = iCAT_task(tasks{i}.A, tasks{i}.J, ...
                                          Qp, ydotbar, tasks{i}.xdotbar, ...
                                          1e-4, 0.01, 10);
           end

           % 3. Last task: residual damping
           [~, ydotbar] = iCAT_task(eye(13), eye(13), Qp, ydotbar, zeros(13,1), 1e-4, 0.01, 10);

           % 4. Split velocities for vehicle and arm
           qdot = ydotbar(1:7);
           v_nu = ydotbar(8:13); % projected on the vehicle frame
       end







       function [v_nu, qdot] = computeICAT(obj, robot, dt)
            % Get current action
            tasks = obj.unified_list;
            tasks_curr_actions = obj.actions{obj.currentAction};
            tasks_prev_actions = obj.actions{obj.previousAction};

            % 1. Update references, Jacobians, activations off all tasks
            for i = 1:length(obj.unified_list)
                tasks{i}.updateReference(robot);
                tasks{i}.updateJacobian(robot);
                tasks{i}.updateActivation(robot);
            end

            % update time
            obj.time = obj.time + dt;
            % compute obj.a_curr and obj.a_prev
            if obj.currentAction ~= obj.previousAction
                obj.a_curr = IncreasingBellShapedFunction(0, 1, 0, 1, obj.time);
                obj.a_prev = DecreasingBellShapedFunction(0, 1, 0, 1, obj.time);
            end

            % 2. Perform ICAT (task-priority inverse kinematics)
            ydotbar = zeros(13,1);
            Qp = eye(13);
            
            for i = 1:length(tasks)
                % 1. Extract the current task object handle (use curly braces)
                current_task_handle = tasks{i}; 
                
                % 2. Check if this handle exists in the current or previous action lists
                % We iterate through the cells and compare handles using '=='
                is_in_curr = any(cellfun(@(x) x == current_task_handle, tasks_curr_actions));
                is_in_prev = any(cellfun(@(x) x == current_task_handle, tasks_prev_actions));
            
                % 3. Apply logic based on booleans
                % 3. Determine activation scalar
                activation_scalar = 0;
                if is_in_curr && is_in_prev
                    activation_scalar = 1;
                elseif is_in_curr
                    activation_scalar = obj.a_curr;
                elseif is_in_prev
                    activation_scalar = obj.a_prev;
                end
                
                % 4. Apply ICAT if active
                if activation_scalar > 0
                    [Qp, ydotbar] = iCAT_task(current_task_handle.A * activation_scalar, current_task_handle.J, ...
                                               Qp, ydotbar, current_task_handle.xdotbar, ...
                                               1e-4, 0.01, 10);
                end
            end

            % 3. Last task: residual damping
            [~, ydotbar] = iCAT_task(eye(13), eye(13), Qp, ydotbar, zeros(13,1), 1e-4, 0.01, 10);

            % 4. Split velocities for vehicle and arm
            qdot = ydotbar(1:7);
            v_nu = ydotbar(8:13); % projected on the vehicle frame
        end

















        function setCurrentAction(obj, name_of_action)
            actionIndex = find(strcmp(name_of_action, string(obj.actions_name)));
            disp(obj.actions_name)
            disp(actionIndex)
            % Switch to a different action
            if actionIndex >= 1 && actionIndex <= length(obj.actions)
                % first save the previous action index
                obj.previousAction = obj.currentAction;
                % then set current action
                obj.currentAction = actionIndex;
                % reset timer
                obj.time = 0;
            else
                error('There is not an action with this name');
            end
        end
    end
end