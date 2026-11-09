classdef TaskVehiclePosition < Task
    %TASKVEHICLEPOSITION Three-dimensional vehicle-position task.
    %
    % The task drives the vehicle origin toward the position stored in robot.wTgv. Its position error, reference velocity and task-space rate are expressed in the world frame, while the commanded vehicle linear velocity contained in v_nu is expressed in the vehicle frame.

    properties
        feedbackGain = 3          % Proportional gain that converts position error into a linear velocity reference [1/s].
        maxLinearVelocity = 0.5   % Maximum absolute component of the linear velocity reference [m/s].
    end

    methods
        function updateReference(obj, robot)
            %UPDATEREFERENCE Build a bounded world-frame position feedback velocity.
            %
            % Input:
            %   robot - UvmsModel containing the current vehicle pose robot.wTv and desired vehicle pose robot.wTgv.
            %
            % Output:
            %   None. Updates obj.xdotbar with the desired world-frame vehicle-position rate [m/s].

            % The position error points from the current vehicle origin to the goal. The feedback gain converts it into a velocity reference, and Saturate preserves its direction while enforcing the configured component bound.
            positionError = robot.wTgv(1:3,4) - robot.wTv(1:3,4);
            obj.xdotbar = Saturate(obj.feedbackGain*positionError, obj.maxLinearVelocity);
        end

        function updateJacobian(obj, robot)
            %UPDATEJACOBIAN Map the UVMS velocity to world-frame vehicle translation.
            %
            % Input:
            %   robot - UvmsModel providing the current vehicle orientation robot.wTv(1:3,1:3) and the number of arm joints.
            %
            % Output:
            %   None. Updates obj.J, a [3x13] matrix with columns ordered as [seven arm joints, vehicle linear velocity, vehicle angular velocity].

            % Arm motion and vehicle angular velocity do not translate the vehicle origin. The rotation wRv maps its body-frame linear velocity into world coordinates.
            wRv = robot.wTv(1:3,1:3);
            obj.J = [zeros(3,numel(robot.q)) wRv zeros(3,3)];
        end

        function updateActivation(obj, robot)
            %UPDATEACTIVATION Keep all three position components fully active.
            %
            % Input:
            %   robot - UvmsModel (unused; retained for the common Task API).
            %
            % Output:
            %   None. Sets obj.A to the [3x3] identity matrix.
            %
            obj.A = eye(3);
        end
    end
end
