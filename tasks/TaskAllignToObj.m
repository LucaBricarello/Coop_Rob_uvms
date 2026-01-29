classdef TaskAllignToObj < Task   
    properties
        theta
        %n;
        %d;
        
        v_d
        v_n
        
        error
    end

    methods
        function updateReference(obj, robot)

            rad_ref = 0;
            wTv = robot.wTv;
            wRv = wTv(1:3,1:3);

            % computing vector that connects robot origin with nodule frame origin (w_lin)
            [w_ang, w_lin] = CartError(robot.wTg , robot.wTv); % where robot.wTg is the tool goal frame (so the nodule frame)
            
            w_k_w = [0; 0; 1];
            w_P = eye(3) - w_k_w * w_k_w';

            w_d = w_P * w_lin;
            obj.v_d = wRv' * w_d;

            v_i_v = [1; 0; 0];

            w_i_v = wRv * v_i_v;

            if norm(w_d) < 1e-7
                return;
            end

            w_nd = w_d / norm(w_d);

            w_n = cross(w_i_v, w_nd);

            sin_theta = norm(w_n);
            cos_theta = w_nd' * w_i_v;

            obj.theta = atan2(sin_theta, cos_theta);

            if sin_theta > 1e-6
                axis_w = w_n / sin_theta; 
            else
                axis_w = [0; 0; 1]; % Asse fittizio per evitare NaN, tanto theta è 0
            end
            
            % Portiamo l'asse unitario nel frame veicolo
            obj.v_n = wRv' * axis_w;

            %obj.v_n = wRv' * w_n;

            obj.xdotbar = - 0.3 * (obj.theta * obj.v_n);
            %obj.xdotbar = - 0.4 * obj.theta;

            obj.xdotbar = Saturate(obj.xdotbar, 0.2);

            obj.error = norm(obj.theta * obj.v_n);
            %obj.error = obj.theta;
        end

        function updateJacobian(obj, robot)

            wRv = robot.wTv(1:3,1:3);
            w_P = diag([1, 1, 0]);
            P_body = wRv' * w_P * wRv;

            %obj.J = obj.v_n' * [zeros(3,7), -(1/norm(obj.v_d)^2)*skew(obj.v_d)*P_body, -eye(3)];

            obj.J = (obj.v_n * obj.v_n' - (obj.theta/sin(obj.theta)) * skew([1;0;0]) * skew(obj.v_d/norm(obj.v_d)) * (eye(3) - (obj.v_n * obj.v_n'))) * [zeros(3,7), -(1/norm(obj.v_d)^2)*skew(obj.v_d)* P_body, -eye(3)];

        end
        
        function updateActivation(obj, robot)

            %obj.A = 1;

            obj.A = eye(3);

        end
    end
end