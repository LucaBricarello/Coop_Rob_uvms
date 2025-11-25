classdef TaskLand < Task   
    properties

    end

    methods
        function updateReference(obj, robot)

            ref_distance = 0;
            
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

            vRw = robot.vTw(1:3, 1:3);

            v_k_w = vRw * [0, 0, 1]';

            J_a = v_k_w' * [zeros(3,7), eye(3), zeros(3,3)];

            obj.J = J_a;

        end
        
        function updateActivation(obj, robot)
            obj.A = 1;
        end
    end
end