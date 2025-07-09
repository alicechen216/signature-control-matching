% =========================================================================
% FILE: simulate_system.m
% =========================================================================
% Simulates the nonlinear dynamical system from Eq. (11) in the paper
% using the first-order Euler method.
%
% INPUTS:
%   U      - Piecewise-constant input signal (vector of size N).
%   mu     - System parameter.
%   theta  - System parameter.
%   sigma  - System parameter.
%   t_grid - Time grid vector (size N+1).
%   Z0     - Initial state.
%
% OUTPUT:
%   Z      - The resulting state trajectory (vector of size N+1).
% =========================================================================
function Z = simulate_system(U, mu, theta, sigma, t_grid, Z0)
    dt = t_grid(2) - t_grid(1);
    N = length(t_grid) - 1;
    Z = zeros(size(t_grid));
    Z(1) = Z0;

    for j = 1:N
        % Euler step for dZ_t = theta*Z_t*(mu - Z_t^2)*dt + sigma*U_t*dt
        dZ = theta * Z(j) * (mu - Z(j)^2) * dt + sigma * U(j) * dt;
        Z(j+1) = Z(j) + dZ;
    end
end
