function [h,theta] = RotToAngleAxis(R)
%EULER REPRESENTATION: Given a tensor rotation matrices this function
% should output the equivalent angle-axis representation values,
% respectively 'theta' (angle), 'h' (axis)
% SUGGESTED FUNCTIONS
% size()
% eye()
% abs()
% det()
% NB: Enter a square, 3x3 proper-orthogonal matrix to calculate its angle
% and axis of rotation. Error messages must be displayed if the matrix
% does not satisfy the rotation matrix criteria.
tollerance = 1e-3;
if  ~isequal(size(R), [3, 3])
    error("ERROR: not a 3-by-3 square matrix");
elseif rank(R) ~= 3
    error("ERROR: R is not full rank");
elseif any(R*transpose(R)-eye(3) > tollerance)
    disp(R*transpose(R)-eye(3));
    error("ERROR: inv(R) is not equal to transpose(R)");
elseif any(abs(transpose(R)*R-eye(3)) > tollerance)
    error("ERROR: inv(R) is not equal to transpose(R)");
elseif  det(R) - 1 > tollerance
    error("ERROR: rotation matrix must have determinant -1 or 1");
end


% Check matrix R to see if its size is 3x3
theta = acos((trace(R)-1)/2);
Skew_R = (R-transpose(R))/2;
a = vex(Skew_R);
if theta ==0
    warning("WARNING: not able to find vector axis since the angle is 0");
    h = [1 0 0];
elseif theta == pi
    % implementation of formula (R = 2h'h-I)
    for i =1 :3
        % calculating i_signed = sqrt((R(i,i)+1)/2)
        
        % [root_pos root_neg] = sqrt((R(i,i)+1)/2);
        % since I have 2 outputs from the sqrt, i decide to take into
        % consideration always the positive root(or, in general, always the first root)
        h(i ) = abs(sqrt((R(i,i)+1)/2));
        for j =1:3
            if j == i 
                continue;
            end
            if h(i) ~= 0
                % the formula would need the sign(h_(i))*sign(R(i,j))*sqrt((R(j,j)+1)/2)
                h(j) = sign(R(i,j))*sqrt((R(j,j)+1)/2);
            end
        end
    end

else
    h = a /sin(theta);
    h =h';
end
end


function a = vex(S_a)
% input: skew matrix S_a (3x3)
% output: the original a vector (3x1)
a = [   S_a(3,2)- S_a(2,3);
    S_a(1,3)- S_a(3,1);
    S_a(2,1)- S_a(1,2)];
a = a/2;

end