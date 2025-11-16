function [psi,theta,phi] = RotToYPR(R)
% Given a rotation matrix the function outputs the relative euler angles
% usign the convention YPR
% Input:
% R rotation matrix
% Output:
% psi angle around z axis (yaw)
% theta angle around y axis (theta)
% phi angle around x axis (phi)
% SUGGESTED FUNCTIONS
    % atan2()
    % sqrt()

    % Checking that R is a rotation matrix
    tollerance = 1e-5;
    if  ~isequal(size(R), [3, 3])
        fprintf("ERROR: not a 3-by-3 square matrix");
        return;
    elseif any(abs(transpose(R)*R-eye(3)) > tollerance)
        fprintf("ERROR: inv(R) is not equal to transpose(R)");
        return;
    elseif  abs(det(R) - 1) > tollerance
        fprintf("ERROR: rotation matrix must have determinant equal to 1");
        return;
    end

    % -------------------------------------------------------------

    theta = atan2(-R(3,1), sqrt(R(1,1)^2 + R(2,1)^2));

    if cos(theta) == 0  %infinite solutions I choose a random one
        psi = 0;
        phi = 0;
        warning("Gimbal lock, singular configuration, infinite solutions\n");
    else
        psi = atan2(R(2,1), R(1,1));
        phi = atan2(R(3,2), R(3,3));
    end
    
end

