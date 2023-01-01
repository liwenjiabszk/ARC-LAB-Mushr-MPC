%% getting_started_test.m

close all;
clear all;
clc;

%% Make the solver
%  You have to generate first the code from c++!!! The following MATLAB
%  script will be auto-generated.
make_acado_solver;

%% Create the structures and set dimensions
%  Those dimensions must match ones defined in the c++ code generator.
N   = 60;
NX  = 3;
NU  = 2;
NY  = 5;
NYN = 3;

% Always use integration from the last node for shifting
mpcInput.shifting.strategy = 0;
% Initialization by a forward simulation.
mpcInput.initialization = 1;

mpcInput.x = zeros(N + 1, NX);
mpcInput.u = zeros(N, NU);
mpcInput.y = zeros(N, NY);
mpcInput.yN = zeros(NYN, 1)';

% In case weighting matrices are not defined, you can use:
%mpcInput.W = eye( NY );
%mpcInput.WN = eye( NYN ) * 5;

mpcInput.x0 = [0,0,0]';

%% Run the simulation

nSteps = 70; %simulation time hence is 70*0.1=7 seconds
state_record =ones(nSteps,NX);
output_record =zeros(nSteps,NU);
sol_time = zeros(nSteps,1);
% Run the simulation and plot horizons
%hf = figure;
for kk = 1: nSteps
    % Call the solver
    mpcInput.x0 = mpcInput.x0';
    mpcInput.y(:,1)= [0.1*((nSteps-1)+[0:0.1:6-0.1])]';
    mpcInput.y(:,2) = 0.5;
    mpcInput.y(:,3) = 0;
    mpcInput.y(:,4) = 0.1;
    mpcInput.y(:,5) = 0;
    mpcInput.yN = [mpcInput.y(end,1),0.5,0];
    %mpcInput.y(:,1:NX) = Myinterpolation(mpcInput.x0(1),mpcInput.x0(2),mpcInput.yN(1),mpcInput.yN(2),mpcInput.yN(3),N,NX);
    %mpcInput.y(:,NX) = 0;
    %mpcInput.y(:,4) = 0.5;
    tic
    mpcOutput = acado_solver( mpcInput );
    state_record(kk,:)=mpcInput.x0;
    output_record(kk,:)=mpcOutput.u(1,:);
    sol_time(kk)=toc;
    % Plot results
%     figure(nSteps);
%     subplot(5, 1, 1);
%         stairs(mpcOutput.x(:, 1), 'b');
%         legend('x');
%     subplot(5, 1, 2);
%         stairs(mpcOutput.x(:, 2), 'b');
%         legend('y');
%     subplot(5, 1, 3);
%         stairs(mpcOutput.x(:, 3), 'b');
%         legend('theta');
%     subplot(5, 1, 4);
%         stairs(mpcOutput.u(:, 1), 'b');
%         legend('v');
%     subplot(5, 1, 5);
%         stairs(mpcOutput.u(:, 2), 'r');
%         legend('delta');
    %pause;
    
    % Prepare for the next step
    mpcInput.x = mpcOutput.x;
    mpcInput.u = mpcOutput.u;
    mpcInput.x0 = mpcOutput.x(2, :)';
    
end;
tspan = [0:0.1:0.1*(nSteps-1)];
figure 
plot(state_record(:,1),state_record(:,2))
legend('Vehicle Trajectory')

figure
subplot(5,1,1)
plot(tspan,state_record(:,1))
xlabel('time (s)')
legend('x')
subplot(5,1,2)
plot(tspan,state_record(:,2))
xlabel('time (s)')
legend('y')
subplot(5,1,3)
plot(tspan,state_record(:,3))
xlabel('time (s)')
legend('theta')
subplot(5,1,4)
plot(tspan,output_record(:,1))
xlabel('time (s)')
legend('v')
subplot(5,1,5)
plot(tspan,output_record(:,2))
xlabel('time (s)')
legend('delta')

disp(['Average solving time per step (one QP) is ', num2str(mean(sol_time)),' seconds'])


function waypts = Myinterpolation(x0,y0,xf,yf,theta_f,N,NX)

    waypts = zeros(N,NX);
    waypts(:,1)= linspace(x0,xf,N);
    waypts(:,2)= linspace(y0,yf,N);
    for i=1:(N/2)
        waypts(i,3) = atan((waypts(i+1,2)-waypts(i,2))/(waypts(i+1,1)-waypts(i,1)));
    end
    waypts(i+1:N,3)=linspace(waypts(i,3),theta_f,N/2);
end











