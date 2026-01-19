classdef TaskAttitude < Task   
    properties
        error % save it as scalar to avoid compatibility issues, if the error is a vector, do the norm
    end

    methods
        function updateReference(obj, robot)
            wRv = robot.wTv(1:3, 1:3);
            wRgv = robot.wTgv(1:3, 1:3);
            
            yaw_curr = atan2(wRv(2,1), wRv(1,1));
            yaw_des  = atan2(wRgv(2,1), wRgv(1,1));
            
            raw_error = yaw_des - yaw_curr;
            
            true_yaw_error = atan2(sin(raw_error), cos(raw_error));

            kp = 0.3; 
            obj.xdotbar = kp * true_yaw_error;

            % limit the requested velocities...
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);

            % save error to check when task is completed
            obj.error = abs(true_yaw_error);
        end

        function updateJacobian(obj, robot)
            obj.J = [zeros(1,7), zeros(1,3), 0, 0, 1];
        end
        
        function updateActivation(obj, robot)
            obj.A = 1;
        end
    end
end