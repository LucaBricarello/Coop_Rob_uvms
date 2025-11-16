classdef TaskMinAltitude < Task   
    properties

    end

    methods
        function updateReference(obj, robot)
            
            if isempty(robot.altitude)
                distance_from_seasfloor = 1 + 3;
            else
                distance_from_seasfloor = 1 + robot.altitude;
            end

            obj.xdotbar = - 0.2 * distance_from_seasfloor;

            % limit the requested velocities...
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        function updateJacobian(obj, robot)
            obj.J = [zeros(1,7), 0, 0, -1, zeros(1,3)];
        end
        
        function updateActivation(obj, robot)
            if isempty(robot.altitude)
                obj.A = DecreasingBellShapedFunction(1.5, 1, 0, 1, 3);
            else
                obj.A = DecreasingBellShapedFunction(1.5, 1, 0, 1, robot.altitude);
            end
        end
    end
end