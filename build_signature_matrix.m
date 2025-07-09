% =========================================================================
% FILE: build_signature_matrix.m
% =========================================================================
% Constructs the signature matrix S as defined by Eq. (8) and Theorem 3.
% It computes the signature S_{0,t}(X) for each time point t in the grid.
%
% INPUTS:
%   X       - The input path matrix [t, V] of size (N+1) x (d+1).
%   indices - Cell array of multi-indices from generate_indices.m.
%
% OUTPUT:
%   S_matrix - The resulting signature matrix of size N x L.
% =========================================================================
function S_matrix = build_signature_matrix(X, indices)
    N = size(X, 1) - 1;
    L = length(indices);
    S_matrix = zeros(N, L);
    
    % A map to quickly find the column index for a given multi-index
    idx_map = containers.Map('KeyType', 'char', 'ValueType', 'double');
    for k = 1:L
        idx_map(mat2str(indices{k})) = k;
    end
    
    % S_prev stores the signature S_{0, t_{j-1}}
    S_prev = zeros(1, L); 
    
    % Iterate through each time interval [t_{j-1}, t_j]
    for j = 1:N 
        delta_X = X(j+1, :) - X(j, :); % Path increment over the interval
        
        % Calculate signature for the current increment using Prop. 4
        sig_increment = zeros(1, L);
        for k = 1:L
            idx = indices{k};
            len_idx = length(idx);
            term = 1 / factorial(len_idx);
            for l_idx = 1:len_idx
                term = term * delta_X(idx(l_idx) + 1);
            end
            sig_increment(k) = term;
        end
        
        % Use Chen's identity (Theorem 2) to compute S_{0, t_j}
        % S_{0,t_j} = S_{0,t_{j-1}} (tensor product) S_{t_{j-1},t_j}
        S_current = S_prev; % Start with S_{0,t_{j-1}}
        for k = 1:L
            idx_k = indices{k};
            len_k = length(idx_k);
            
            % Add the term from the increment signature itself
            S_current(k) = S_current(k) + sig_increment(k);
            
            % Add the cross-terms from the tensor product
            for p = 1:(len_k - 1)
                idx_prefix = idx_k(1:p);
                idx_suffix = idx_k(p+1:end);
                
                k_prefix = idx_map(mat2str(idx_prefix));
                k_suffix = idx_map(mat2str(idx_suffix));

                S_current(k) = S_current(k) + S_prev(k_prefix) * sig_increment(k_suffix);
            end
        end

        S_matrix(j, :) = S_current;
        S_prev = S_current; % Update for the next iteration
    end
end
