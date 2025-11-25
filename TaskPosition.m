classdef TaskPosition < Task   
    properties
        error % save it as scalar to avoid compatibility issues, if the error is a vector, do the norm
    end

    methods
        function updateReference(obj, robot)
            [w_ang, w_lin] = CartError(robot.wTgv , robot.wTv);
            obj.xdotbar = - 0.2 * w_lin;
            % limit the requested velocities...
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.2);
            % save error to check when task is completed
            obj.error = norm(w_lin);
        end
        function updateJacobian(obj, robot)
            vJdw = [zeros(3,7), -eye(3), zeros(3)];

            wRv = robot.wTv(1:3, 1:3);

            wJdw = wRv * vJdw;

            obj.J = wJdw;
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(3);
        end
    end
end