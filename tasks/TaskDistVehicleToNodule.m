classdef TaskDistVehicleToNodule < Task    
    properties
        error;
    end
    
    methods
        function updateReference(obj, robot)
            [w_ang, w_lin] = CartError(robot.wTg , robot.wTv);

            w_lin_xy = w_lin(1:2); % analysing only the distance on the xy plane since we may want to navigate higher and then land near the object

            obj.xdotbar = -0.2 * w_lin_xy;
            % limit the requested velocities...
            obj.xdotbar(1:2) = Saturate(obj.xdotbar(1:2), 0.2);
            % save error to check when task is completed
            obj.error = norm(w_lin_xy);
        end

        function updateJacobian(obj, robot)
            vJdw = [zeros(3,7), -eye(3), zeros(3)];

            wRv = robot.wTv(1:3, 1:3);

            vJdw = wRv * vJdw;

            % removing last row of the Jacobian since I want to control
            % only x pos and y pos

            vJdw = vJdw(1:2, :);

            obj.J = vJdw;
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(2) * DecreasingBellShapedFunction(1.5, 2, 1, 0, obj.error);
        end
    end
end
