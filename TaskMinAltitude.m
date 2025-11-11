classdef TaskMinAltitude < Task   
    properties

    end


    methods
        function updateReference(obj, robot)
            
            if isempty(robot.eta(3))
                distance_from_seasfloor = -39 + (-37);
                disp("robot.altitude: ")
                disp(robot.eta(3))
            else
                distance_from_seasfloor = -39 + robot.eta(3);
            end

            obj.xdotbar = 0.6 * distance_from_seasfloor;

            % limit the requested velocities...
            obj.xdotbar = Saturate(obj.xdotbar, 0.6);
        end
        function updateJacobian(obj, robot)
            obj.J = [zeros(1,7), 0, 0, -1, zeros(1,3)];
        end
        
        function updateActivation(obj, robot)
            obj.A = DecreasingBellShapedFunction(-39.5, -39, 0, 1, robot.eta(3));
            if isempty(robot.altitude)
                obj.A = DecreasingBellShapedFunction(-39.5, -39, 0, 1, -37);
            else
                obj.A = DecreasingBellShapedFunction(-39.5, -39, 0, 1, robot.eta(3));
            end
        end
    end
end