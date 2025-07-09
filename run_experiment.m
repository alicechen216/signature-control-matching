% =========================================================================
% FILE: run_experiment.m
% =========================================================================
% This is the main script to execute the numerical experiment described in
% Section 5.2 of the paper "On the role of the signature transform in
% nonlinear systems and data-driven control".
% It performs the following steps:
% 1. Sets up the experiment parameters.
% 2. Trains a signature-based model of the system dynamics.
% 3. Solves the output-matching control problem using `fmincon`.
% 4. Analyzes and plots the results.
%
% To run: Place all .m files in the same directory and execute this script.
% =========================================================================

clear; clc; close all;

%% ===== 1. Setup Experiment Parameters (from Sec. 5.2) =====
disp('1. Setting up experiment...');
% System parameters from Eq. (11)
mu = 1;
theta = 1;
sigma = 1;
Z0 = 0;

% Time grid
T = 1.0;
dt = 0.05;
t_grid = (0:dt:T)';
N = length(t_grid) - 1;

% Signature and Model parameters
M = 4;       % Truncation order
d = 1;       % Number of exogenous inputs (U_t)
N_train = 40;% Number of training trajectories

% Generate multi-indices for all signature terms up to order M
indices = generate_indices(d, M);
L = length(indices); % Total number of signature features
fprintf('Signature truncation M=%d, leads to L=%d features.\n', M, L);


%% ===== 2. Train the Signature Model (from Sec. 4.1) =====
disp('2. Training the signature model...');

% Pre-allocate matrices for linear regression (Eq. 9)
S_total = zeros(N * N_train, L);
Z_total = zeros(N * N_train, 1);

for i = 1:N_train
    % Generate a random piecewise-linear input path V_t for training.
    % The paper generates values in the range [0, 3].
    V_train_increments = (rand(N, 1) * 3 / N);
    V_train = [0; cumsum(V_train_increments)];
    U_train = V_train_increments / dt;
    
    % Simulate the system to get the corresponding state trajectory
    Z_train = simulate_system(U_train, mu, theta, sigma, t_grid, Z0);
    
    % The input path X_t includes time, so X_t = [t, V_t]
    X_train = [t_grid, V_train];
    
    % Build the signature matrix S for this trajectory based on Eq. (8)
    S_train = build_signature_matrix(X_train, indices);
    
    % Stack the results for the full regression problem
    row_start = (i-1)*N + 1;
    row_end = i*N;
    S_total(row_start:row_end, :) = S_train;
    Z_total(row_start:row_end) = Z_train(2:end) - Z0;
end

% Solve for the linear coefficients beta_hat using least squares
% This corresponds to Eq. (9) in the paper.
beta_hat = S_total \ Z_total;
disp('Training complete. Model coefficients beta_hat have been estimated.');


%% ===== 3. Output-Matching Control (from Sec. 4.2) =====
disp('3. Setting up and running the output-matching control problem...');

% Generate a target trajectory to be tracked
V_true_increments = (rand(N, 1) * 3 / N);
U_true = V_true_increments / dt;
Z_target = simulate_system(U_true, mu, theta, sigma, t_grid, Z0);

% Define the target increments vector Z_bar as per Proposition 7
Z_target_increments = [Z_target(2) - Z0; diff(Z_target(2:end))];

% Define the optimization objective function handle
% This function implements the cost from the nonlinear program (Eq. 10)
objective_func = @(delta_V) optimization_objective(delta_V, t_grid, indices, beta_hat, Z_target_increments, Z0);

% Setup fmincon options. 'sqp' is a good choice for this type of problem.
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp', 'MaxFunctionEvaluations', 10000, 'StepTolerance', 1e-8);

% As per the paper, initialize with a perturbation of the true input
initial_guess = V_true_increments + 0.1 * randn(N, 1); 

% Run the nonlinear optimization to find the optimal input increments (Delta)
disp('Solving the nonlinear program (Eq. 10) using fmincon...');
[delta_V_opt, cost] = fmincon(objective_func, initial_guess, [], [], [], [], [], [], [], options);
fprintf('Optimization finished with final cost: %f\n', cost);

%% ===== 4. Analyze and Plot Results =====
disp('4. Analyzing results and plotting...');

% Reconstruct the optimal control input U_opt from the optimized increments
U_opt = delta_V_opt / dt;

% Simulate the system with the calculated optimal input to get the achieved trajectory
Z_achieved = simulate_system(U_opt, mu, theta, sigma, t_grid, Z0);

% Calculate the Fit score as defined in Eq. (12)
fit_score = 100 * (1 - norm(Z_achieved - Z_target) / norm(Z_target));
fprintf('Output-matching complete. Final Fit Score: %.2f%%\n', fit_score);

% Plot the results, replicating Figure 3 from the paper
figure('Name', 'Signature-Based Output Matching', 'NumberTitle', 'off');
plot(t_grid, Z_target, 'b-', 'LineWidth', 2.5, 'DisplayName', 'Target trajectory');
hold on;
plot(t_grid, Z_achieved, 'color', [0.9290, 0.6940, 0.1250], 'LineWidth', 2, 'LineStyle', '--', 'DisplayName', 'Achieved via signature control');
grid on;
box on;
xlabel('Time [s]', 'FontSize', 12);
ylabel('System state Z(t)', 'FontSize', 12);
title('Signature-Based Output-Matching Control', 'FontSize', 14);
legend('Location', 'best', 'FontSize', 12);
set(gca, 'FontSize', 10);
