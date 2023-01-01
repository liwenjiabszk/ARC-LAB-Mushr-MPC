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
N   = 10;
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

%% Define Trajectory

% %Square Trajectory
% nStepperLine = 120;
% Length = 6;
% 
% x1 = linspace(0,Length,nStepperLine);
% y1= zeros(1,nStepperLine);
% theta1=zeros(1,nStepperLine);
% 
% x2 = Length*ones(1,nStepperLine);
% y2 = linspace(0,Length,nStepperLine);
% theta2=zeros(1,nStepperLine) + pi/2;
% 
% x3 = linspace(Length,0,nStepperLine);
% y3 = zeros(1,nStepperLine) + Length;
% theta3 = zeros(1,nStepperLine) + pi;
% 
% x4 = zeros(1,nStepperLine);
% y4 = linspace(Length,0,nStepperLine);
% theta4 = zeros(1,nStepperLine) + 3/2*pi;
% xr = [x1 x2 x3 x4];
% yr = [y1 y2 y3 y4];
% thetar = [theta1 theta2 theta3 theta4];
% 
% xr = [xr xr xr];
% yr = [yr yr yr];
% thetar = [thetar (thetar+2*pi) (thetar+4*pi)];
% 
% mpcInput.x0 = [0,0,0]';

%Circle Trajectory
nStepperLoop = 360;
Radius = 5;
Vr = (20*Radius*pi)/nStepperLoop;
degree = 2*pi*linspace(1,nStepperLoop,nStepperLoop)/nStepperLoop;
xr = Radius*cos(degree);
yr = Radius*sin(degree);
thetar = degree + pi/2;

xr = [xr xr xr];
yr = [yr yr yr];
thetar = [thetar (thetar+2*pi) (thetar+4*pi)];

mpcInput.x0 = [6,0,pi/2]';

% %Sin Trajectory
% nStepperLoop = 120;
% Amplitude = 3;
% xr = 2*pi*linspace(1,nStepperLoop,nStepperLoop)/nStepperLoop;
% yr = Amplitude*sin(xr);
% thetar = pi/2*cos(xr);
% 
% xr = [xr xr+2*pi xr+4*pi];
% yr = [yr yr yr];
% thetar = [thetar thetar thetar];
% 
% mpcInput.x0 = [0,-1,pi/2]';

% %8-fig Trajectory
% nStepperSeg = 60;
% UnitLength = 3;
% x1 = zeros(1,nStepperSeg);
% y1 = linspace(0,UnitLength,nStepperSeg);
% 
% degree2 = 2*pi*linspace(1,nStepperSeg*3,nStepperSeg)/(nStepperSeg*3)*3/4;
% x2 = UnitLength*cos(degree2)-UnitLength;
% y2 = UnitLength*sin(degree2)+UnitLength;
% 
% x3 = linspace(-UnitLength,UnitLength,2*nStepperSeg);
% y3 = zeros(1,2*nStepperSeg);
% 
% degree4 = 2*pi*linspace(nStepperSeg*1,nStepperSeg*(-2),nStepperSeg)/(nStepperSeg*3)*3/4;
% x4 = UnitLength*cos(degree4)+UnitLength;
% y4 = UnitLength*sin(degree4)-UnitLength;
% 
% x5 = zeros(1,nStepperSeg);
% y5 = linspace(-UnitLength,0,nStepperSeg);
% 
% xr = [x1 x2 x3 x4 x5];
% yr = [y1 y2 y3 y4 y5];
% 
% xr = [xr xr xr];
% yr = [yr yr yr];
% thetar = zeros(size(xr));
% 
% mpcInput.x0 = [UnitLength/2,0,pi/2]';

%% Run the simulation

nSteps = 600; %simulation time hence is 70*0.1=7 seconds
state_record =ones(nSteps,NX);
output_record =zeros(nSteps,NU);
sol_time = zeros(nSteps,1);
% Run the simulation and plot%Square Trajectory horizons
%hf = figure;

for kk = 1: nSteps
    % Call the solver
    mpcInput.x0 = mpcInput.x0';
    mpcInput.y(:,1) = xr(kk:kk+N-1);
    mpcInput.y(:,2) = yr(kk:kk+N-1);
    mpcInput.y(:,3) = thetar(kk:kk+N-1);
    mpcInput.y(:,4) = 0;
    mpcInput.y(:,5) = 0;
%   mpcInput.yN = [mpcInput.y(end,1),0.5,0];
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
plot(state_record(:,1),state_record(:,2),'r--','linewidth',2)
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
