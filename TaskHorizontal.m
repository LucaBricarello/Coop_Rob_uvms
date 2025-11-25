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
        theta
        n
    end


    methods
        function updateReference(obj, robot)
            % computing angle between v frame and w frame since v has to be
            % horizontal wrt w
            wRv = robot.wTv(1:3, 1:3);
            [obj.n, obj.theta] = RotToAngleAxis(wRv);

            %dot_theta = 0.1 * (0.1 - obj.theta);

            %w_omega_v_w = wRv * robot.v_nu(4:6);

            obj.xdotbar = 0.1 * (0.1 - obj.theta);
            % limit the requested velocities...
            obj.xdotbar = Saturate(obj.xdotbar, 0.1);
        end
        function updateJacobian(obj, robot)
            vJdw = [zeros(3,7), zeros(3), eye(3)];

            wRv = robot.wTv(1:3, 1:3);

            wJdw = wRv * vJdw;

            obj.J = obj.n * wJdw;
        end
        
        function updateActivation(obj, robot)
            obj.A = IncreasingBellShapedFunction(0.1,0.2,0,1,obj.theta);
        end
    end
end