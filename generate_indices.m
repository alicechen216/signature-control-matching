% =========================================================================
% FILE: generate_indices.m
% =========================================================================
% Generates all multi-indices for the signature up to a given truncation
% order M. For an input path in R^(d+1), an index is a sequence
% (i_1, ..., i_k) where each i is in {0, 1, ..., d}.
%
% INPUTS:
%   d - Number of exogenous inputs (dimension of U).
%   M - Truncation order of the signature.
%
% OUTPUT:
%   indices - A cell array containing all multi-indices as vectors.
% =========================================================================
function indices = generate_indices(d, M)
    indices = {};
    % Start with indices of length 1
    current_level_indices = num2cell(0:d);
    
    for k = 1:M
        % Add the current level of indices to the total list
        indices = [indices, current_level_indices];
        
        % Generate the next level of indices
        next_level_indices = {};
        for i = 1:length(current_level_indices)
            base_index = current_level_indices{i};
            for j = 0:d
                next_level_indices{end+1} = [base_index, j];
            end
        end
        current_level_indices = next_level_indices;
    end
end
