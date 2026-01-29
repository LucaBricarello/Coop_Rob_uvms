classdef SimulationLogger < handle
    properties
        t            % time vector
        q            % joint positions
        q_dot        % joint velocities
        eta          % vehicle pose
        v_nu         % vehicle velocity
        a            % task activations (diagonal only)
        a_combined   % task activations multiplied by action activation
        xdotbar_task % reference velocities for tasks (cell array)
        robot        % robot model
        task_set     % set of tasks
        action_manager % reference to action manager

        altitude     % robot altitude
        task_errors  % errors of tasks

        error

        t_switch_1
        t_switch_2
        t_switch_3
    end

    methods
        function obj = SimulationLogger(maxLoops, robotModel, task_set, action_manager)
            obj.robot = robotModel;
            obj.task_set = task_set;
            if nargin > 3
                obj.action_manager = action_manager;
            end

            obj.t = zeros(1, maxLoops);
            obj.q = zeros(7, maxLoops);
            obj.q_dot = zeros(7, maxLoops);
            obj.eta = zeros(6, maxLoops);
            obj.v_nu = zeros(6, maxLoops);

            obj.error = zeros(3, maxLoops);
            
            obj.altitude = zeros(1, maxLoops);
            obj.task_errors = zeros(length(task_set), maxLoops);

            % Store the diagonal of each activation matrix
            maxDiagSize = max(cellfun(@(t) size(t.A,1), task_set));
            obj.a = zeros(maxDiagSize, maxLoops, length(task_set));
            obj.a_combined = zeros(maxDiagSize, maxLoops, length(task_set));

            % Initialize cell array to store task reference velocities
            obj.xdotbar_task = cell(length(task_set), maxLoops);
        end

        function update(obj, t, loop)
            % Store robot state
            obj.t(loop) = t;
            obj.q(:, loop) = obj.robot.q;
            obj.q_dot(:, loop) = obj.robot.q_dot;
            obj.eta(:, loop) = obj.robot.eta;
            obj.v_nu(:, loop) = obj.robot.v_nu;
            if isempty(obj.robot.altitude)
                obj.altitude(loop) = 3;
            else
                obj.altitude(loop) = obj.robot.altitude;
            end

            [w_ang, w_lin] = CartError(obj.robot.wTgv , obj.robot.wTv);
            obj.error(:, loop) = w_lin;

            % Store task activations (diagonal only) and reference velocities
            for i = 1:length(obj.task_set)
                diagA = diag(obj.task_set{i}.A);           % extract diagonal
                obj.a(1:length(diagA), loop, i) = diagA;
                obj.xdotbar_task{i, loop} = obj.task_set{i}.xdotbar;
                
                if isprop(obj.task_set{i}, 'error') && ~isempty(obj.task_set{i}.error)
                    obj.task_errors(i, loop) = norm(obj.task_set{i}.error);
                end

                % Calculate combined activation if action_manager is present
                if ~isempty(obj.action_manager)
                    current_task = obj.task_set{i};
                    tasks_curr_actions = obj.action_manager.actions{obj.action_manager.currentAction};
                    tasks_prev_actions = obj.action_manager.actions{obj.action_manager.previousAction};
                    
                    is_in_curr = any(cellfun(@(x) x == current_task, tasks_curr_actions));
                    is_in_prev = any(cellfun(@(x) x == current_task, tasks_prev_actions));
                    
                    activation_scalar = 0;
                    if is_in_curr && is_in_prev
                        activation_scalar = 1;
                    elseif is_in_curr
                        activation_scalar = obj.action_manager.a_curr;
                    elseif is_in_prev
                        activation_scalar = obj.action_manager.a_prev;
                    end

                    if i == 1 % TASK STOP MOVE
                        if is_in_curr
                            activation_scalar = 1;
                        elseif is_in_prev && ~is_in_curr
                            activation_scalar = 0;
                        end
                    end
                    
                    obj.a_combined(1:length(diagA), loop, i) = diagA * activation_scalar;
                end
            end
        end

        function plotAll(obj)
            % Example plotting for robot state
            figure(1);
            set(gcf, 'Color', 'w');
            subplot(2,1,1);
            plot(obj.t, obj.q, 'LineWidth', 1);
            legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
            subplot(2,1,2);
            plot(obj.t, obj.q_dot, 'LineWidth', 1);
            legend('qd_1','qd_2','qd_3','qd_4','qd_5','qd_6','qd_7');

            figure(2);
            set(gcf, 'Color', 'w');
            subplot(2,1,1);
            plot(obj.t, obj.eta, 'LineWidth', 1);
            legend('x','y','z','roll','pitch','yaw');
            subplot(2,1,2);
            plot(obj.t, obj.v_nu, 'LineWidth', 1);
            legend('xdot','ydot','zdot','omega_x','omega_y','omega_z');

            % Optional: plot task activations
            figure(3);
            set(gcf, 'Color', 'w');
            for i = 1:size(obj.a,3)
                subplot(size(obj.a,3),1,i);
                plot(obj.t, squeeze(obj.a(:, :, i))', 'LineWidth', 1);
                title(['Task ', num2str(i), ' Activations (diagonal)']);
            end

            figure(4);
            set(gcf, 'Color', 'w');
            plot(obj.t, obj.error, 'LineWidth', 1);
            legend('x','y','z','roll','pitch','yaw');

            % Plot combined task activations
            if ~isempty(obj.action_manager)
                figure(5);
                set(gcf, 'Color', 'w');
                for i = 1:size(obj.a_combined,3)
                    subplot(ceil(size(obj.a_combined,3)/2),2,i);
                    if i == 1
                        plot(obj.t, squeeze(obj.a_combined(:, :, i))', 'LineWidth', 1);
                        title('Task Stop Move');
                    elseif i == 2
                        plot(obj.t, squeeze(obj.a_combined(1, :, i))', 'LineWidth', 1);
                        title('Task Min Altitude');
                        ylim([0, 1]);
                    elseif i == 3
                        plot(obj.t, squeeze(obj.a_combined(1, :, i))', 'LineWidth', 1);
                        title('Task Horizontal Attitude');
                    elseif i == 4
                        plot(obj.t, squeeze(obj.a_combined(1, :, i))', 'LineWidth', 1);
                        title('Task Dist to Obj');
                    elseif i == 5
                        plot(obj.t, squeeze(obj.a_combined(1:3, :, i))', 'LineWidth', 1);
                        title('Task Allign to Obj');
                    elseif i == 6
                        plot(obj.t, squeeze(obj.a_combined(1, :, i))', 'LineWidth', 1);
                        title('Task Land');
                    elseif i == 7
                        plot(obj.t, squeeze(obj.a_combined(1:3, :, i))', 'LineWidth', 1);
                        title('Task Position');
                    elseif i == 8
                        plot(obj.t, squeeze(obj.a_combined(1:3, :, i))', 'LineWidth', 1);
                        title('Task Orientation');
                    elseif i == 9
                        plot(obj.t, squeeze(obj.a_combined(:, :, i))', 'LineWidth', 1);
                        title('Task Tool');
                    end
                    xline(0, '-r', 'A1'); 
                    xline(obj.t_switch_1, '-r', 'A2'); 
                    xline(obj.t_switch_2, '-r', 'A3');
                    xline(obj.t_switch_3, '-r', 'A4');
                    xlabel('Time [s]'); ylabel('Act [0,1]');
                    grid on;
                end
            end
            
            % Min Altitude Task Analysis
            figure(6);
            set(gcf, 'Color', 'w');
            sgtitle('Min Altitude Task Analysis');

            subplot(1,2,1);
            plot(obj.t, squeeze(obj.a(1, :, 2))', 'LineWidth', 1);
            title('Task Min Altitude Activation');
            xlabel('Time [s]'); ylabel('Activation [0-1]');
            grid on;

            subplot(1,2,2);
            plot(obj.t, obj.altitude, 'LineWidth', 1);
            yline(1.5, '--g', '1.5m');
            yline(1.1, '--b', '1.1m');
            yline(0.75, '--r', '0.75m');
            title('Robot Altitude');
            xlabel('Time [s]'); ylabel('Altitude [m]');
            legend('Current Altitude', 'Upper Activation Threshold', 'Lower Activation Threshold', 'Min Limit');
            grid on;

            % Horizontal Attitude Task Analysis
            figure(7);
            set(gcf, 'Color', 'w');
            sgtitle('Horizontal Attitude Task Analysis');
            
            subplot(1,2,1);
            plot(obj.t, squeeze(obj.a(1, :, 3))', 'LineWidth', 1);
            title('Task Horizontal Activation');
            xlabel('Time [s]'); ylabel('Activation [0-1]');
            grid on;
            
            subplot(1,2,2);
            plot(obj.t, obj.task_errors(3, :), 'LineWidth', 1);
            title('Task Horizontal Error');
            xlabel('Time [s]'); ylabel('Error Norm');
            grid on;
            
            % Task Dist V to Obj Task Analysis
            figure(8);
            set(gcf, 'Color', 'w');
            sgtitle('Task Dist V to Obj Task Analysis');

            subplot(1,2,1);
            plot(obj.t, squeeze(obj.a_combined(1, :, 4))', 'LineWidth', 1);
            title('Task Dist V to Obj Activation');
            xline(obj.t_switch_1, '-r', 'A2'); 
            xlabel('Time [s]'); ylabel('Activation [0-1]');
            grid on;
            
            subplot(1,2,2);
            plot(obj.t, obj.task_errors(4, :), 'LineWidth', 1);
            yline(1.8, '--b', '1.8m');
            yline(1.5, '--r', '1.5m');
            xline(obj.t_switch_1, '-r', 'A2'); 
            title('Task Dist V to Obj Error');
            xlabel('Time [s]'); ylabel('Distance [m]');
            legend('Distance Error', 'Upper Activation Threshold', 'Lower Activation Threshold');
            grid on;
        end
    end
end