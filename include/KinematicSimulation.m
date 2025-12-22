%% Kinematic Simulation function
%
% Inputs
% - q current robot configuration
% - q_dot joints velocity
% - ts sample time
% - q_min lower joints bound
% - q_max upper joints bound
%
% Outputs
% - q new joint configuration
function [q_new] = KinematicSimulation(q,q_dot, dt, q_min, q_max)
    n = size(q,1)
    q_new = zeros(n, 1); % Initialize new joint configuration
    qi = q + q_dot * dt;
    for i=1:n
        if qi(i) < q_min(i)
            qi(i) = q_min(i); % Clamp to lower bound
        elseif qi(i) > q_max(i)
            qi(i) = q_max(i); % Clamp to upper bound
        end
        q_new(i) = qi(i); % Update new joint configuration
    end
end