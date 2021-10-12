close all; clear all;
% Create dynamic functions
symbolic_dynamics_pendulum();

% Load in trajectory for swing up
data = load('swingup-trajectory.mat');
states = data.states; inputs = data.inputs; dt = data.dt; parameters = data.parameters;

% Define weighting matrices
n_states = size(states,2); n_inputs = size(inputs,2);
Q_k = 0.001*eye(n_states); % We care most about reaching the end goal of swinging up
R_k = 0.001*eye(n_inputs);

Q_T = 100*eye(n_states);

K_ltvlqr = ltvlqr(states,inputs,@calc_A_disc,@calc_B_disc,Q_k,R_k,Q_T,dt,parameters);

% Simulate with a perturbation
new_states = zeros(size(inputs,1)+1,size(states,2));
init_state = states(1,:)'; % Get the initial state
current_state = init_state + [-pi/8;0];
new_states(1,:) = current_state';
for ii=1:size(inputs,1)
    % Get the current gains and compute the feedforward and
    % feedback terms
    current_feedback = reshape(K_ltvlqr(ii,:,:),n_inputs,n_states)*(current_state-states(ii,:)');
    current_input = inputs(ii,:)' + current_feedback;
    
    % simualte forward
    next_state = calc_f_disc(current_state,current_input,dt,parameters);
    % Store states and inputs
    new_states(ii+1,:) = next_state';
    % Update the current state
    current_state = next_state;
end
figure(1);
h1 = plot(states(:,1),states(:,2),'k-');
hold on
h2 = plot(new_states(:,1),new_states(:,2),'b--');
legend([h1,h2],"Reference Trajectory","Perturbed Trajectory");
xlabel('$$\theta$$');
ylabel('$$\dot{\theta}$$');

figure(2);
animate_pendulum(new_states,dt)

