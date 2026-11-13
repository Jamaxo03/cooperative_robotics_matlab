classdef TaskAltitude < Task
    % The task keeps the vehicle horizontal (|theta_w - theta_v| <= 0.2). 
    % Everything will be expressed in the vehicle frame.

    properties
        feedbackGain = 0.5;
        delta = 0.5;
        distMax = 0;
        type = "";
    end

    methods
        % Constructor to make the task modular
        function obj = TaskAltitude(d, type)
            obj.distMax = d;
            obj.type = type;
            if obj.type == "equality"
                obj.delta = 0;
            end
        end

        function updateReference(obj, robot)
            %UPDATEREFERENCE Build a bounded Cartesian feedback velocity.
            
            % Distance from the vehicle to ground: we use robot.altitude
            % Compute xdotbar
            if isnan(robot.altitude)
                obj.xdotbar = 0;                 % no measurement yet: request nothing
            else
                obj.xdotbar = Saturate(obj.feedbackGain*((obj.distMax + obj.delta) - robot.altitude), 0.2);
            end
            end

        function updateJacobian(obj, robot)
            %UPDATEJACOBIAN Build the tool Jacobian for arm and vehicle motion.
            %
            % Input:
            %   robot - UvmsModel providing q and the transforms vTb, vTe, vTt and eTt.
            %
            % Output:
            %   None. Updates obj.J, a [6x13] matrix with columns ordered as [seven arm joints, six vehicle velocities]. Rows follow the [angular; linear] convention used by obj.xdotbar.

            % Theta_dot = n' * omega
            wRv = robot.wTv(1:3, 1:3);
            obj.J = [zeros(1,7), [0, 0, 1] * wRv, zeros(1,3)];
        end

        function updateActivation(obj, robot)
            %UPDATEACTIVATION.
            %
            % Input:
            %   robot - UvmsModel (unused; retained for the common Task API).
            %
            % Output:
            %   None. Sets obj.A according to the inequality constraints.
            %
            % The activation function fades smoothly in the interval 2-2.5
            if isnan(robot.altitude)
                obj.A = 0;                       % no measurement: task off
            elseif obj.type == "equality"
                obj.A = 1;
            else
                obj.A = DecreasingBellShapedFunction(obj.distMax, obj.distMax+obj.delta, 0, 1, robot.altitude);
            end
        end
    end
end