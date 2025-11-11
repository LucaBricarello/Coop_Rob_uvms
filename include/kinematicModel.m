%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        J % Jacobian
    end

    methods
        % Constructor to initialize the geomModel property
        function self = kinematicModel(gm)
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, self.gm.jointNumber);
            else
                error('Not enough input arguments (geometricModel)')
            end
        end
        function updateJacobian(self)
        %% Update Jacobian function
        % The function update:
        % - J: end-effector jacobian matrix

            for i = 1 : self.gm.jointNumber
                
                b_T_i = self.gm.getTransformWrtBase(i);
                b_R_i = b_T_i(1:3, 1:3);

                b_T_e = self.gm.getTransformWrtBase(self.gm.jointNumber);

                e_r_i = b_T_e(1:3, 4) - b_T_i(1:3, 4);      % already w.r.t. frame <b>

                e_r_i_skew_symm = [0, -e_r_i(3), e_r_i(2);
                                   e_r_i(3), 0, -e_r_i(1);
                                   -e_r_i(2), e_r_i(1), 0];

                if self.gm.jointType(i) == 0        % Revolute

                    Ja = b_R_i * [0; 0; 1];
                    %Ja = b_R_i(:,3);
                    Jl = e_r_i_skew_symm' * b_R_i * [0; 0; 1];

                else        % Prismatic

                    Ja = [0; 0; 0];
                    Jl = b_R_i * [0; 0; 1];

                end

                self.J(1:3, i) = Ja;
                self.J(4:6, i) = Jl;

            end

            if ~exist('gm.eTt','var')
                
                e_r_te = [0.2; 0; 0];
                bTe = self.gm.getTransformWrtBase(7);
                bRe = bTe(1:3, 1:3);
                b_r_te = bRe * e_r_te;
                b_r_te_skew= [0 -b_r_te(3) b_r_te(2);
                    b_r_te(3) 0 -b_r_te(1);
                    -b_r_te(2) b_r_te(1) 0];
                S = [eye(3), 0*eye(3);
                    b_r_te_skew' ,eye(3);]; % rigid body jacobian

                self.J = S * self.J;
            end

        end
    end
end

