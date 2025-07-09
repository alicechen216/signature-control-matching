% =========================================================================
% FILE: optimization_objective.m
% =========================================================================
% This function defines the objective for the nonlinear program in Eq. (10).
% It calculates the squared error between the target state increments and
% the increments predicted by the signature model.
%
% INPUTS:
%   delta_V             - Optimization variable: increments of the input V.
%   t_grid              - The time grid.
%   indices             - Cell array of multi-indices.
%   beta_hat            - The estimated model coefficients.
%   Z_target_increments - The vector of target state increments (Z_bar).
%   Z0                  - The initial state.
%
% OUTPUT:
%   cost                - The scalar value of the objective function.
% =========================================================================
function cost = optimization_objective(delta_V, t_grid, indices, beta_hat, Z_target_increments, Z0)
    
    % Reconstruct the full input path X from the optimization variables
    V_path = [Z0; cumsum(delta_V)];
    X_path = [t_grid, V_path];
    
    % Build the signature matrix S for the current candidate path
    S_current = build_signature_matrix(X_path, indices);
    
    % Construct S_bar based on Proposition 7. This represents the change
    % in signature features at each step, S_{0,t_j} - S_{0,t_{j-1}}.
    L = length(indices);
    S_with_prev = [zeros(1, L); S_current(1:end-1, :)];
    S_bar = S_current - S_with_prev;
    
    % Predict the state increments using the trained model
    Z_pred_increments = S_bar * beta_hat;
    
    % Calculate the squared error (the cost to be minimized)
    cost = sum((Z_target_increments - Z_pred_increments).^2);
end
