classdef TaskDistVehicleToNodule < Task    
    properties
        error;
        n;
    end
    
    methods
        function updateReference(obj, robot)
            [w_ang, w_lin] = CartError(robot.wTg , robot.wTv);

            w_lin_xy = w_lin(1:2); % analysing only the distance on the xy plane since we may want to navigate higher and then land near the object

            obj.error = norm(w_lin_xy);

            wTv = robot.wTv;
            wRv = wTv(1:3,1:3);
            w_lin(3) = 0;
            n_w = w_lin/norm(w_lin);
            obj.n = wRv' * n_w;

            obj.xdotbar = - 0.25 * obj.error;
            % limit the requested velocities...
            obj.xdotbar(1) = Saturate(obj.xdotbar(1), 0.2);
        end

        function updateJacobian(obj, robot)
            wRv = robot.wTv(1:3,1:3);
            w_P = diag([1, 1, 0]);
            P_body = wRv' * w_P * wRv;

            obj.J = [zeros(1,7), obj.n'*(-eye(3))*P_body, zeros(1,3)];
        end
        
        function updateActivation(obj, robot)
            obj.A = IncreasingBellShapedFunction(1.5, 1.8, 0, 1, obj.error);
        end
    end
end
