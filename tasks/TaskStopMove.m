classdef TaskStopMove < Task   
    properties

    end

    methods

        function updateReference(obj, robot)
            obj.xdotbar = zeros(6,1);
        end

        function updateJacobian(obj, robot)
            J_lin = [zeros(3,7), eye(3), zeros(3)];
            J_ang = [zeros(3,7), zeros(3), eye(3)];

            obj.J = [J_lin; J_ang];
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(6);
        end
    end
end