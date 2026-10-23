classdef TaskHorizontalAttitude < Task
    %TASKTOOL Six-dimensional Cartesian pose task for the UVMS tool.
    %
    % The task drives the tool frame vTt toward the goal frame vTg. Both frames are expressed relative to the vehicle, so the resulting error, reference velocity and Jacobian use a consistent vehicle-frame representation. All six components are permanently active.

    properties
        feedbackGain = 0.2        % Proportional gain that converts Cartesian error into a velocity reference [1/s].
        maxAngularVelocity = 0.2  % Maximum absolute component of the angular velocity reference [rad/s].
        maxLinearVelocity = 0.2   % Maximum absolute component of the linear velocity reference [m/s].

        thetaMin = 0.1;
        thetaMax = 0.2;
        theta = 0;
        n = zeros(3,1);
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

            v_kw = robot.vTw(1:3,1:3) * [0; 0; 1];
            v_kv = [0; 0; 1];
            rho = ReducedVersorLemma(v_kw, v_kv);

            obj.theta = norm(rho);
            if obj.theta > 1e-6
                obj.n = rho / obj.theta;
            else
                obj.n = zeros(3,1);
            end

            % Limit angular and linear parts independently while preserving the direction within each three-dimensional subvector.
            % obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), obj.maxAngularVelocity);
            % obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), obj.maxLinearVelocity);

            obj.xdotbar = obj.feedbackGain * (obj.thetaMin - obj.theta);
            obj.xdotbar = Saturate(obj.xdotbar, obj.maxAngularVelocity);
        end

        function updateJacobian(obj, robot)
            %UPDATEJACOBIAN Build the tool Jacobian for arm and vehicle motion.
            %
            % Input:
            %   robot - UvmsModel providing q and the transforms vTb, vTe, vTt and eTt.
            %
            % Output:
            %   None. Updates obj.J, a [6x13] matrix with columns ordered as [seven arm joints, six vehicle velocities]. Rows follow the [angular; linear] convention used by obj.xdotbar.

            obj.J = [zeros(1,7) zeros(1,3) obj.n'];
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
            obj.A = IncreasingBellShapedFunction(obj.thetaMin, obj.thetaMax, 0, 1, obj.theta);
        end
    end
end
