%% Kinematic Simulation function



function q = KinematicSimulation(q, q_dot, ts, q_min, q_max)
% Inputs
% - q current robot configuration
% - q_dot joints velocity
% - ts sample time
% - q_min lower joints bound
% - q_max upper joints bound
%
% Outputs
% - q new joint configuration

    for i = 1 : length(q)
        q_temp = q(i) + q_dot(i) * ts;
    
        if q_temp <= q_max(i) && q_temp >= q_min(i)
            q(i) = q_temp;
        else
            if q_temp > q_max(i)
                q(i) = q_max(i);
            else
                q(i) = q_min(i);
            end
        end
    end

end