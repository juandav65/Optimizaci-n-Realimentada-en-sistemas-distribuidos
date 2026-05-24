clear all; close all; clc;

%% Simulation Parameters
simTime = 100; % Simulation time (sufficient for asymptotic convergence)
dt = 0.1; % Time step for Euler integration
N_steps = round(simTime / dt);
gamma = [1, 1, 10, 10]; % γ₁, γ₂ cost function gains (equation 4)
N_agents = 4; % 4 agents for square formation
R = .2; % Sensing radius (for neighbors)
dim = 3; % State dimension [x, y, θ] for each agent
epsilon = 0.1;

%% Formation Definition (Square - 4 agents)
% Based on Section I-B: Graph Theory preliminaries and Section II
% Incidence matrix B
% Represents the formation topology (connections between agents)
B = [1,  0,  0, -1;  % Agent 1 
     -1, 1,  0,  0;  % Agent 2 
     0, -1,  1,  0;  % Agent 3 
     0,  0, -1,  1]; % Agent 4 

% Desired displacement vector d
% Defines the target relative positions between connected agents
d = reshape(...
    [1,  0, -1,  0;    % X displacements: agent2-1=1, agent3-2=0, etc.
     0,  1,  0, -1],... % Y displacements
    [], 1);

% Adjacency matrix A
A = [0, 1, 0, 1;  % Agent 1 
     1, 0, 1, 0;  % Agent 2 
     0, 1, 0, 1;  % Agent 3 
     1, 0, 1, 0]; % Agent 4 
 
% Calculate algebraic connectivity λ₂ (Lemma III.2 and Theorem III.2)
% λ₂ characterizes graph connectivity - crucial for convergence
lambdas = eig(B*B.'); % Eigenvalues of Laplacian L_G = BB^T
lambda2 = min(lambdas(lambdas > 1e-5)); % Second smallest eigenvalue

%% Target (Fixed position) and Initial Conditions
% τ in paper notation - target location known to all agents
t = [3; 3]; % Target position τ
% Random initial conditions near origin
initial_position_boundaries = ...
    [-.1, .1;     % x ∈ [-0.1, 0.1]
	 -.1, .1;     % y ∈ [-0.1, 0.1]
     -pi/2, pi/2];    % φ ∈ [-π/2, π/2] (orientation)
x = rand(3, N_agents) ...
    .* repmat(diff(initial_position_boundaries, [], 2), 1, N_agents) ...
    + repmat(min(initial_position_boundaries, [], 2), 1, N_agents);
x = x(:); % State vectorization [x₁, y₁, φ₁, x₂, y₂, φ₂, ...]^T

%% Initialize arrays for storing data
% For convergence analysis and plotting (Section IV)
trajectory = zeros(2, N_agents, N_steps); % Position trajectories
formationCosts = zeros(N_steps, 1); % Formation cost Φ_d (equation 4a)
targetCosts = zeros(N_steps, 1); % Target cost Φ_τ (equation 4b)

%% Disabled parameters (no obstacles, no safety features)
% Problem simplification - without obstacles
params = [0, 0, 0, 0, 0]; % All features disabled

%% Main Simulation Loop
% Implements the control scheme from Figure 1 of the paper
u = out_implemented(x);
fprintf('Simulating square formation...\n');
for k = 1:N_steps
    % Get current positions (output) - equation (6b): y = g(x,u) = r
    y = out_implemented(x);
    
    % Store trajectory for plotting (as in Fig. 2-4 of the paper)
    trajectory(:, :, k) = reshape(y, 2, N_agents);
    
    % Calculate costs for analysis - equation (4)
    % z = B₂^T r - d (formation error)
    z = kron(B.', eye(2)) * y - d;
    formationCosts(k) = 0.5 * norm(z)^2; % Φ_d(r) = γ₁/2 * ‖z‖²
    targetCosts(k) = 0.5 * norm(y - repmat(t, N_agents, 1))^2; % Φ_τ(r) = γ₂/2 * ‖r - τ̄_N‖²
    
    % Controller - equation (7): ˙u = -ε[γ₁(L_G₂r - B₂d) + γ₂(r - τ̄_N)]
    W = zeros(N_agents, 1); % No obstacles detected
    ODelta = zeros(2*N_agents, 1); % No obstacle positions
    u = ctrl_implemented(gamma, R, B, d, t, W, ODelta, y, y, params);
    
    % Dynamics - equations (1) and (3): ˙x = f(x,u) with low-level control
    dxdt = dyn_implemented_velocity(x, u, N_agents, 0);
    x = x + dxdt * dt; % Euler integration
    
    % Progress indicator
    if mod(k, N_steps/10) == 0
        fprintf('Progress: %d%%\n', round(100*k/N_steps));
    end
end

%% Analysis and Plotting
fprintf('Plotting results...\n');

% 1. Plot formation evolution 
figure('Position', [100, 100, 1200, 900]);
T_plot = [1, floor(N_steps/4), floor(N_steps/2), floor(3*N_steps/4), N_steps];
plotFormation(trajectory, A, T_plot);
title('Square Formation Evolution', 'Interpreter', 'latex', 'FontSize', 16);
grid on;

% 2. Plot costs 
figure('Position', [100, 100, 1200, 400]);
subplot(1,2,1);
semilogy((1:N_steps)*dt, formationCosts, 'LineWidth', 2);
xlabel('Time', 'Interpreter', 'latex');
ylabel('Formation Cost', 'Interpreter', 'latex');
title('Formation Cost Convergence', 'Interpreter', 'latex');
grid on;

subplot(1,2,2);
semilogy((1:N_steps)*dt, targetCosts, 'LineWidth', 2);
xlabel('Time', 'Interpreter', 'latex');
ylabel('Target Cost', 'Interpreter', 'latex');
title('Target Cost Convergence', 'Interpreter', 'latex');
grid on;

% 3. Compare tuning 
optRadius = 0.5; % Desired formation radius
optTargetCost = 0.1; % Desired target cost threshold
compareTuning(formationCosts, targetCosts, trajectory, t, optRadius, optTargetCost, N_agents, {'Default Tuning'});

fprintf('Simulation complete!\n');

%% Helper Functions

function dudt = ctrl_implemented(gamma, R, B, d, t, W, ODelta, y, u, params)
% Implements the distributed control law from equation (7)
% Based on Section III-A: Feedback optimisation as a distributed control law
    N_agents = numel(y) / 2;
    % Equation (7): ˙u = -ε[γ₁(L_G₂r - B₂d) + γ₂(r - τ̄_N)]
    dudt = ...
        -gamma(1) * (kron(B*B.',eye(2)) * y - kron(B, eye(2)) * d) ... % Formation control: γ₁(L_G₂r - B₂d)
        -gamma(2) * (y - repmat(t, N_agents, 1)); % Target tracking: γ₂(r - τ̄_N)
    % Note: This implementation is distributed - each agent only uses local information
    % from its neighbors (as demonstrated in Section III-A)
end

function dxdt = dyn_implemented_velocity(x, u, na, nw)
% Implements the plant dynamics with low-level control
% Based on equations (1), (2), (3) and Lemma II.1
    
    N = numel(x) / 3; % N agents with state [x, y, φ]
    k = [.1, .1]; % Low-level control gains (k_i in equation 3)
    velSat = [-Inf, Inf]; % No velocity saturation

    % Low-level controller - equation (3)
    % v_i = k_i ξ_i cos(φ_i), ω_i = k_i (cos(φ_i) + 1) sin(φ_i)
    ubar = zeros(2,N);
    states = reshape(x,3,N); % Reorganize states: [x_i; y_i; φ_i]
    delta = reshape(u,2,N) - states(1:2,:); % ξ_i = u_i - r_i
    e = vecnorm(delta,2,1); % ‖ξ_i‖
    theta = atan2(delta(2,:), delta(1,:)) - states(3,:); % φ_i = atan2(ξ_{b_i}, ξ_{a_i}) - θ_i

    % Linear velocity control (v_i)
    ubar(1,:) = k(1) * e .* cos(theta);
    
    % Angular velocity control (ω_i) - robust implementation
    thetag = theta(theta >= 1e-3);
    eg = e(theta >= 1e-3);
    ubar(2,:) = (ubar(1,:) ./ e + e.^2).* sin(theta);
    ubar(2,theta >= 1e-3) = (ubar(1,theta >= 1e-3) ./ eg + eg.^2 .* sin(thetag) ./ thetag) .* sin(thetag);

    % Input saturation 
    vel = max(velSat(1), min(velSat(2), ubar(1,:)));
    angVel = ubar(2,:);

    % Dynamics - equation (1): ˙a_i = v_i cos(θ_i), ˙b_i = v_i sin(θ_i), ˙θ_i = ω_i
    dir = [cos(states(3,:)); sin(states(3,:))]; % Direction vector
    dxdt = reshape([repmat(vel,2,1) .* dir; angVel], [], 1); % Vectorization
end

function y = out_implemented(x)
% Output map - equation (6b): y = g(x,u) = r
% Returns only agent positions
    dim = 3;
    state = reshape(x,dim,[]);
    y = reshape(state(1:2,:), [], 1); % [x₁, y₁, x₂, y₂, ...]^T
end

function plotFormation(trajectory,adjacency,T)
% Formation plotting function 
    cmap = flipud(jet); % Colormap for temporal evolution

    % Plot agent trajectories (colored lines)
    z = (1:size(trajectory,3)).';
    for i=1:size(trajectory,2)
        x=squeeze(trajectory(1,i,:));
        y=squeeze(trajectory(2,i,:));
        hp = patch([x.' NaN], [y.' NaN], 0);
        set(hp,'cdata', [z.' NaN], 'edgecolor','interp','facecolor','none');
        hold on;
        scatter(x,y,1,z,'filled');
    end

    % Time selection for plotting 
    if numel(T) == 1
        step = ceil(size(trajectory,3) / T);
        steps = 1:step:size(trajectory,3)-step;
    else
        steps = T;
    end
        
    % Plot formations at specific times
    for t = steps
        formation_plot(trajectory, adjacency, t, cmap(ceil(size(cmap,1) * t / size(trajectory,3)),:));
    end
    formation_plot(trajectory, adjacency, size(trajectory,3), cmap(end,:));

    colormap(cmap)
    c = colorbar('southoutside', 'Ticks', []);
    c.Label.String = 'Simulation time';
    c.Label.Interpreter = 'latex';
    xlabel('X position', 'interpreter', 'latex');
    ylabel('Y position', 'interpreter', 'latex');
end

function formation_plot(trajectory, adjacency, t, color)
% Formation plot at a specific time
    markersize=14;
    textsize=10;

    % Draw formation graph edges
    for i=1:size(trajectory,2)
        for j=i+1:size(trajectory,2)
            if adjacency(i,j)==1
                pi=trajectory(:,i,t);
                pj=trajectory(:,j,t);
                line([pi(1),pj(1)], [pi(2),pj(2)], 'linewidth', 2, 'color', color);
            end
        end
    end

    % Draw agents with labels
    for i=1:size(trajectory,2)
        x=trajectory(1,i,t);
        y=trajectory(2,i,t);
        plot(x, y, 'o', 'MarkerEdgeColor', color, 'MarkerFaceColor', color, 'markersize', 7)
        plot(x, y, 'o', ...
            'MarkerSize', markersize,...
            'linewidth', 2,...
            'MarkerEdgeColor', color,...
            'markerFaceColor', [1 1 1]);
        text(x, y, num2str(i),...
            'color', color, 'FontSize', textsize, 'horizontalAlignment', 'center', 'FontName', 'times');
    end
end

function compareTuning(formationCosts,targetCosts,trajectories,target,optRadius,optTargetCost,N_agents,labels)
% Function for tuning comparison (sensitivity analysis)
% Based on Lemma III.2 and Theorem III.2 about γ₁, γ₂ effects
    figure('Position', [10 10 900 450]);
    for i = 1:size(formationCosts,2)
        semilogy(formationCosts(:,i));
        hold on;
    end
    xlabel('Simulation time', 'interpreter', 'latex');
    ylabel('$$\log(\Phi_d(t) / \gamma_2)$$', 'interpreter', 'latex');
    title('Formation cost over simulation time', 'interpreter', 'latex');
    legend(labels, 'interpreter', 'latex')
    set(gca,'FontSize',12);

    figure('Position', [10 10 900 450]);
    for i = 1:size(targetCosts,2)
        semilogy(targetCosts(:,i));
        hold on;
    end
    xlabel('Simulation time', 'interpreter', 'latex');
    ylabel('$$\log(\Phi_{\tau}(t) / \gamma_2)$$', 'interpreter', 'latex');
    yline(optTargetCost, '--'); % Reference line for desired target cost
    labels{end+1} = 'Desired';
    title('Target cost over simulation time', 'interpreter', 'latex');
    legend(labels, 'interpreter', 'latex')
    set(gca,'FontSize',12);

    figure('Position', [10 10 900 450]);
    for i = 1:size(trajectories, 4)
        trajectory = trajectories(:,:,:,i);
        radius = max(reshape(vecnorm(reshape(trajectory,2,[]) - target),N_agents,[]));
        plot(radius);
        hold on;
    end
    yline(optRadius, '--'); % Reference line for formation radius
    xlabel('Simulation time', 'interpreter', 'latex');
    ylabel('Distance from target', 'interpreter', 'latex');
    title('Distance from target over simulation time', 'interpreter', 'latex');
    legend(labels, 'interpreter', 'latex')
    set(gca,'FontSize',12);
end