%% Kinematic Model Class - GRAAL Lab
classdef cartesianControl < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        k_a
        k_l
    end

    methods
        % Constructor to initialize the geomModel property
        function self = cartesianControl(gm,angular_gain,linear_gain)
            if nargin > 2
                self.gm = gm;
                self.k_a = angular_gain;
                self.k_l = linear_gain;
            else
                error('Not enough input arguments (cartesianControl)')
            end
        end
        function [x_dot]=getCartesianReference(self,bTg)
            %% getCartesianReference function
            % Inputs :
            % bTg : goal frame
            % Outputs :
            % x_dot : cartesian reference for inverse kinematic control
            bTt = self.gm.getToolTransformWrtBase();
            bRt = bTt(1:3,1:3);
            tTb = inv(bTt);

            tTg = tTb * bTg;

            t_r_gt = tTg(1:3,4);

            tRg = tTg(1:3,1:3);
            [h, theta] = RotToAngleAxis(tRg);
            t_rho_tg = h * theta;
            t_rho_tg = t_rho_tg';

            b_r_gt = bRt * t_r_gt;
            b_rho_tg = bRt * t_rho_tg;

            error_vector(1:3,1) = b_rho_tg;
            error_vector(4:6,1) = b_r_gt;

            lambda = eye(6);
            lambda(1:3, 1:3) = lambda(1:3, 1:3) * self.k_a;
            lambda(4:6, 4:6) = lambda(4:6, 4:6) * self.k_l;

            x_dot = lambda * error_vector;
        end
    end
end

