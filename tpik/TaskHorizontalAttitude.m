classdef TaskHorizontalAttitude < Task
    %TASKHORIZONTALATTITUDE Summary of this class goes here
    %   Detailed explanation goes here

    properties
        feedbackGain = 3          % Proportional gain 
        maxVelocity = 0.2   % Maximum absolute component of the reference [rad/s].
        theta
        w_rho
        theta_max = 0.2
        delta = 0.1
    end

    methods
        function updateReference(obj, robot)
            %UPDATEREFERENCE Build a bounded world-frame position feedback velocity.
            %
            % Input:
            %   robot - UvmsModel
            %
            % Output:
            %   None. Updates obj.xdotbar.
 
            obj.w_rho = ReducedVersorLemma([0 0 1]', robot.wTv(1:3,3));
            obj.theta = norm(obj.w_rho);
            obj.xdotbar = Saturate(obj.feedbackGain*((obj.theta_max - obj.delta) - obj.theta), obj.maxVelocity);
        end

        function updateJacobian(obj, robot)
            %UPDATEJACOBIAN Map the UVMS velocity to world-frame vehicle translation.
            %
            % Input:
            %   robot - UvmsModel 
            %
            % Output:
            %   None. Updates obj.J, a [1x13] matrix with columns ordered as [seven arm joints, vehicle linear velocity, vehicle angular velocity].

            
            if (obj.theta ~= 0)
                w_n = obj.w_rho / obj.theta;
                obj.J = w_n'*[zeros(3,numel(robot.q)) zeros(3,3) robot.wTv(1:3,1:3)];
            else
                obj.J = zeros(1, numel(robot.q) + 6);
            end
        end

        function updateActivation(obj, robot)
            %UPDATEACTIVATION Keep all three position components fully active.
            %
            % Input:
            %   robot - UvmsModel (unused; retained for the common Task API).
            %
            % Output:
            %   None. Sets obj.A to an increasing bell shaped function
            %
            obj.A = IncreasingBellShapedFunction(obj.theta_max - obj.delta, obj.theta_max, 0, 1, obj.theta);
        end
    end
end