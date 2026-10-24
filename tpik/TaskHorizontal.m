classdef TaskHorizontal < Task
    % The task keeps the vehicle horizontal (|theta_w - theta_v| <= 0.2). 
    % Everything will be expressed in the vehicle frame.

    properties
        feedbackGain = 0.5;  
        thetaMax = 0.2;  
        delta = 0.1;
        rho = zeros(3,1);  
        theta = 0;
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

            % Orientation z of the robot
            v_kv = [0; 0; 1];
            % Compute orientation z of the world wrt the vehicle frame
            v_kw = robot.vTw(1:3,1:3) * [0; 0; 1];
            % Compute misalignment vector and its norm (theta)
            obj.rho   = ReducedVersorLemma(v_kw, v_kv);
            obj.theta = norm(obj.rho);
            % Compute xdotbar as gain(theta_w - theta_v)
            obj.xdotbar = Saturate(obj.feedbackGain*((obj.thetaMax-obj.delta) - obj.theta), 0.2);
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
            if obj.theta > 1e-6
                n = obj.rho/obj.theta; 
            else
                n = zeros(3,1); 
            end
            obj.J = [zeros(1,7), zeros(1,3), n'];
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
            % The activation function fades smoothly in the interval 0.1-0.2
            obj.A = IncreasingBellShapedFunction(obj.thetaMax-obj.delta, obj.thetaMax, 0, 1, obj.theta);
        end
    end
end
