classdef TaskHorizontal < Task   
    properties
       theta;
        n;
    end

    methods
        function updateReference(obj, robot)

            rad_ref = 0;
            wTv = robot.wTv;
            wRv = wTv(1:3,1:3);

            w_kw = [0,0,1]';
            v_kv = [0,0,1]';
            w_kv = wRv * v_kv;

            axis_world = cross(w_kw, w_kv);

            sin_theta = norm(axis_world);
            cos_theta = w_kv' * w_kw; % scalar product

            obj.theta = atan2(sin_theta, cos_theta);

            if sin_theta < 1e-6
                n_world = [0;0;1]; 
            else
                n_world = axis_world / sin_theta;
            end

            obj.n = wRv' * n_world;

            obj.xdotbar = 0.3 *(rad_ref - obj.theta);

            obj.xdotbar = Saturate(obj.xdotbar, 0.1);
        end

        function updateJacobian(obj, robot)

            obj.J = [zeros(1,7), zeros(1,3), obj.n'*eye(3)];

        end
        
        function updateActivation(obj, robot)

            obj.A = IncreasingBellShapedFunction(0, 0.1, 0, 1, abs(obj.theta));

        end
    end
end