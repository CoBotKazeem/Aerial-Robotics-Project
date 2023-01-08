function sdot = quadEOM(t, s, controlhandle, trajhandle, params)
% INPUTS:
% t             - 1 x 1, time
% s             - 13 x 1, state vector = [x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r]
% controlhandle - function handle of your controller
% trajhandle    - function handle of your trajectory generator
% params        - struct, output from sys_params() and whatever parameters you want to pass in
%
% OUTPUTS:
% sdot          - 13 x 1, derivative of state vector s
% convert state to quad stuct for control
current_state = stateToQd(s);

% Get desired_state
desired_state = trajhandle(t, current_state);

% get control outputs
[F, M] = controlhandle(t, current_state, desired_state, params);

% compute derivative
sdot = quadEOM_readonly(t, s, F, M, params);

end
