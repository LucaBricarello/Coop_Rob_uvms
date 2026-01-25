classdef TaskAllignToObj < Task   
    properties
        theta;
        n;
        d;
        
        error;
    end

    methods
        function updateReference(obj, robot)

            rad_ref = 0;
            wTv = robot.wTv;
            wRv = wTv(1:3,1:3);

            % computing vector that connects robot origin with nodule frame origin (w_lin)
            [w_ang, w_lin] = CartError(robot.wTg , robot.wTv); % where robot.wTg is the tool goal frame (so the nodule frame)
            
            % Projecting 3d vector on xy horizontal plane
            w_lin(3) = 0;

            obj.d = wRv' * w_lin;

            w_axisw = w_lin; % not transposed since it is already a 3x1 vector
            v_iv = [1,0,0]';
            w_iv = wRv * v_iv;

            axis_world = cross(w_axisw, w_iv);

            sin_theta = norm(axis_world);
            cos_theta = w_iv' * w_axisw; % scalar product

            obj.theta = atan2(sin_theta, cos_theta);

            if sin_theta < 1e-6
                n_world = [1;0;0]; 
            else
                n_world = axis_world / sin_theta;
            end

            obj.n = wRv' * n_world;

            %obj.xdotbar = - 0.3 *(rad_ref*obj.n - obj.theta*obj.n);
            obj.xdotbar = - 0.3 *(rad_ref - obj.theta);

            obj.xdotbar = Saturate(obj.xdotbar, 0.1);

            obj.error = rad_ref - obj.theta;
        end

        function updateJacobian(obj, robot)

            %obj.J = [zeros(1,7), zeros(1,3), obj.n'*eye(3)];

            % since n is in the world frame its orientation does not depend
            % on the orientation of the robot
            obj.J = obj.n' * [zeros(3,7), -(1/norm(obj.d)^2)*skew(obj.d), zeros(3,2), [0;0;-1]];

            %obj.J = (obj.n * obj.n' - (obj.theta/sin(obj.theta)) * skew([1;0;0]) * skew(obj.d/norm(obj.d)) * (eye(3) - (obj.n * obj.n'))) * [zeros(3,7), -(1/norm(obj.d)^2)*skew(obj.d), -eye(3)];
            %obj.J = (obj.n * obj.n' - (obj.theta/sin(obj.theta)) * skew([1;0;0]) * skew(obj.d/norm(obj.d)) * (eye(3) - (obj.n * obj.n'))) * [zeros(3,7), -(1/norm(obj.d)^2)*skew(obj.d), zeros(3,2), [0;0;-1]];

        end
        
        function updateActivation(obj, robot)

            obj.A = 1;

            %obj.A = eye(3);

        end
    end
end