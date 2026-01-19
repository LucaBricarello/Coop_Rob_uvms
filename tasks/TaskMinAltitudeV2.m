classdef TaskMinAltitudeV2 < Task   
    properties

    end

    methods
        function updateReference(obj, robot)

            ref_distance = 1.5;
            
            if isempty(robot.altitude)
                fake_altitude = 3;
                distance_error = ref_distance - fake_altitude;
            else
                distance_error = ref_distance - robot.altitude;
            end

            obj.xdotbar = 0.2 * distance_error;

            % limit the requested velocities...
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        function updateJacobian(obj, robot)
            %wRv = robot.wTv(1:3, 1:3);
            %v_K = [0 0 1]';
            %J_linearvel = wRv * [zeros(3,7), -(v_K * v_K') , zeros(3,3)];
            %w_K = wRv * v_K;
            %obj.J = w_K' * J_linearvel

            %obj.J = [zeros(1,7), 0, 0, -1, 0, 0, 0];

            %wRv = robot.wTv(1:3, 1:3);
            %v_K = [0 0 1]';
            %w_K = wRv * v_K;
            %J_linearvel = [zeros(1,7), w_K'*(-w_K*w_K'), zeros(1,3)];
            %obj.J = J_linearvel

            vRw = robot.vTw(1:3, 1:3);

            v_k_w = vRw * [0, 0, 1]';

            J_a = v_k_w' * [zeros(3,7), eye(3), zeros(3,3)];

            obj.J = J_a;

        end
        
        function updateActivation(obj, robot)
            if isempty(robot.altitude)
                obj.A = DecreasingBellShapedFunction(1.5, 2, 0, 1, 3);
            else
                obj.A = DecreasingBellShapedFunction(1.5, 2, 0, 1, robot.altitude);
            end
        end
    end
end