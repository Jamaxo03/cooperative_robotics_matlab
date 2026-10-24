classdef TaskMove < Task
    % The task drives the vehicle frame wTv toward the vehicle goal frame wTgv. 
    % Both frames are expressed relative to the world, so the resulting error, 
    % reference velocity and Jacobian use a consistent world-frame representation. 
    
    properties
        feedbackGain = 0.2
        maxLin = 0.5;
    end

    methods
        function updateReference(obj, robot)
            %UPDATEREFERENCE Build a bounded Cartesian feedback velocity.
            %
            % Input:
            %   robot - UvmsModel
            %
            % Output:
            %   None. Updates obj.xdotbar as linear velocity, expressed in the world frame.

            w_err = robot.vehicleGoalPosition - robot.eta(1:3); % Compute distance between vehicle and goal
            obj.xdotbar = Saturate(obj.feedbackGain * w_err, obj.maxLin); % Define xdotbar as gain(x_gv/w - x_v/w) and saturate it
        end

        function updateJacobian(obj, robot)
            %UPDATEJACOBIAN Build the tool Jacobian for arm and vehicle motion.
            %
            % Input:
            %   robot - UvmsModel providing q and the transforms vTb, vTe, vTt and eTt.
            %
            % Output:
            %   None. Updates obj.J, a [6x13] matrix with columns ordered as [seven arm joints, six vehicle velocities]. 
            
            wRv = robot.wTv(1:3,1:3);
            obj.J = [zeros(3,7), wRv, zeros(3,3)];
        end

        function updateActivation(obj, robot)
            %UPDATEACTIVATION Keep all Cartesian components fully active.
            %
            % Input:
            %   robot - UvmsModel
            %
            % Output:
            %   None. Sets obj.A to the [3x3] identity matrix.
            %#ok<INUSD>
            obj.A = eye(3);
        end
    end
end
