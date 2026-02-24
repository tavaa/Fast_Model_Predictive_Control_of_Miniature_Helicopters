%% GENERATE PWC REF

function [x_ref, u_ref, t_steps] = GeneratePWC_reference(traj_obj, T_sim, Ts, model, options)
% Generates PWC reference using nonlinear model integration
% 
% INPUTS:
%   traj_obj  -> trajectory object (ShapeCircle, Spiral, etc.)
%   T_sim     -> simulation duration
%   Ts        -> sampling time
%   model     -> HelicopterModel object
%   options   -> ode45 options
%
% OUTPUTS:
%   x_ref     -> 10 x N_steps state matrix
%   u_ref     -> 4  x N_steps input matrix
%   t_steps   -> time vector

    % Time vector
    t_steps = 0:Ts:T_sim;
    N_steps = length(t_steps);

    % Preallocate
    x_ref = zeros(10, N_steps);
    u_ref = zeros(4,  N_steps);

    % Initial condition from flatness
    [z0, dz0, ddz0] = traj_obj.get_flat_outputs(0);
    [x0, ~] = FlatnessMap.map(z0, dz0, ddz0);

    x_ref(:,1) = x0;
    x_curr = x0;

    % PWC integration loop
    for k = 1:N_steps-1

        t_now = t_steps(k);

        % Sample flat outputs
        [z, dz, ddz] = traj_obj.get_flat_outputs(t_now);
        [~, u_k] = FlatnessMap.map(z, dz, ddz);

        % Store input
        u_ref(:,k) = u_k;

        % Integrate nonlinear dynamics with constant input
        tspan = [t_now, t_now + Ts];

        [~, x_sol] = ode45(@(t,x) model.dynamics(t, x, u_k, zeros(10,1)), ...
                           tspan, x_curr, options);

        x_curr = x_sol(end,:)';
        x_ref(:,k+1) = x_curr;
    end

    % Repeat last input 
    u_ref(:,N_steps) = u_ref(:,N_steps-1);

end