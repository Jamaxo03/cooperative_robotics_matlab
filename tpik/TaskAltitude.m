classdef TaskAltitude < Task

    properties
        feedbackGain = 0.2        % Proportional gain that converts Cartesian error into a velocity reference [1/s].
        maxLinearVelocity = 0.2   % Maximum absolute component of the linear velocity reference [m/s].
        h_min = -39.5;
        delta = 0.5;
    end

    methods
        function updateReference(obj, robot)
            %UPDATEREFERENCE Build a bounded Cartesian feedback velocity.
            %
            % Input:
            %   robot - UvmsModel with current tool transform robot.vTt and desired tool transform robot.vTg.
            %
            % Output:
            %   None. Updates obj.xdotbar as [angular; linear] velocity, expressed in the vehicle frame.
            
            obj.xdotbar = obj.feedbackGain * (obj.h_min + obj.delta - robot.eta(3));

            % Limit angular and linear parts independently while preserving the direction within each three-dimensional subvector.
            obj.xdotbar = Saturate(obj.xdotbar, obj.maxLinearVelocity);
        end

        function updateJacobian(obj, robot)
            %UPDATEJACOBIAN Build the tool Jacobian for arm and vehicle motion.
            %
            % Input:
            %   robot - UvmsModel providing q and the transforms vTb, vTe, vTt and eTt.
            %
            % Output:
            %   None. Updates obj.J, a [1x13] matrix with columns ordered as [seven arm joints, six vehicle velocities]. Rows follow the [angular; linear] convention used by obj.xdotbar.

            obj.J = [zeros(1,7) robot.wTv(3,1:3), zeros(1,3)];
        end

        function updateActivation(obj, robot)
            %UPDATEACTIVATION Keep all Cartesian components fully active.
            %
            % Input:
            %   robot - UvmsModel (unused; retained for the common Task API).
            %
            % Output:
            %   None. Sets obj.A to the [6x6] identity matrix.
            %#ok<INUSD>
            obj.A = DecreasingBellShapedFunction(obj.h_min, obj.h_min + obj.delta, 0, 1, robot.eta(3));
        end
    end
end