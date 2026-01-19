%classdef TaskHorizontal < Task   
%    properties
%        psi
%        theta
%        phi
%    end
%
%    methods
%        function updateReference(obj, robot)
%            % computing angle between v frame and w frame since v has to be
%            % horizontal wrt w
%            wRv = robot.wTv(1:3, 1:3);
%            [obj.psi, obj.theta, obj.phi] = RotToYPR(wRv);
%
%            obj.xdotbar = 0.1 * ([pi/2; pi/2] - [obj.phi; obj.theta]);
%            % limit the requested velocities...
%            obj.xdotbar = Saturate(obj.xdotbar, 0.1);
%        end
%        function updateJacobian(obj, robot)
%
%            L_inv = [1, sin(obj.phi)*tan(obj.theta), cos(obj.phi)*tan(obj.theta);
%                     0, cos(obj.phi), -sin(obj.phi);
%                     0, sin(obj.phi)/cos(obj.theta), cos(obj.phi)/cos(obj.theta)];
%
%            obj.J = [zeros(2,7), zeros(2,3), - L_inv(1:2 , 1:2), zeros(2,1)];
%        end
%        
%        function updateActivation(obj, robot)
%            obj.A = [IncreasingBellShapedFunction(0.1,0.2,0,1,obj.phi), 0;
%                     0, IncreasingBellShapedFunction(0.1,0.2,0,1,obj.theta)];
%        end
%    end
%end



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

            obj.A = IncreasingBellShapedFunction(0.1, 0.2, 0, 1, abs(obj.theta));

        end
    end
end