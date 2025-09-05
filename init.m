close all
clc
clear all

N = Simulink.Parameter;
N.Value = 14;  % or your desired number of links
N.CoderInfo.StorageClass = 'ExportedGlobal';
% Number of snake robot links
m  = 0.406;                                   % Weight of snake robot link
l  = 0.0525;                                  % Radius of snake robot link
g  = 9.81;                                    % Gravitational acceleration
diameter = 0.40;                              % Diameter of a pipeline
diameterInfluence = 0.10;                     % Auxiliary constant
pipeLength = 3;                               % Length of a pipeline
d = diameter - 2*l;                           % Theoretical diameter for snake robot links
shotsNumber = 50;                             % Auxiliary variable for animation
dt = 0.01;                                    % Time increment [s]
ct = 0.15;                                   % Viscous friction coefphicient of a ground in tangental direction
cn = 0.3;                                    % Viscous friction coefphicient of a ground in normal direction
ut = 0.15;                                    % Coulomb friction coefphicient of a ground in tangental direction
un = 0.3;                                     % Coulomb friction coefphicient of a ground in normal direction
ctPipe = 0.8;                                 % Viscous friction coefphicient of a pipeline in tangental direction
utPipe = 0.2;                                 % Coulomb friction coefphicient of a pipeline in tangental direction
umax = 3;                                     % Maximum snake robot link torque
qmax = 400*dt;                                % Contact parameter
Erub = 400000;                                % Contact parameter
vrub = 0.49;                                  % Contact parameter                        
friction = 1;                                 % Choice of friction:   0 - Coulomb, 1 - viscous
% contact = 1;                                % Choice of side walls contact: 0 - without contact, 1 - with contact
minLinkVel = 0.001;                           % Auxiliary variable of snake robot link angular velocity 
% Controler parameters:
kp  = 25;                                     % Gain for position controller
kd  = 25;                                     % Gain for velocity controller
n_nodi = N.Value-1;
mu = 3;
A = zeros(n_nodi);  % Initialize a square matrix of size N.Value

for i = 1:n_nodi
    if i == 1
        A(i, i) = -mu;  % First row, first column
        if n_nodi > 1
            A(i, i + 1) = mu;  % First row, second column
        end
    elseif i == n_nodi
        A(i, i - 1) = mu;  % Last row, second to last column
        A(i, i) = -mu;     % Last row, last column
    else
        A(i, i - 1) = mu;  % Diagonal below
        A(i, i) = -2 * mu;  % Main diagonal
        A(i, i + 1) = mu;  % Diagonal above
    end
end

B = zeros(n_nodi, n_nodi - 1);  % Initialize a matrix B of size n_nodi x (n_nodi - 1)

for i = 1:n_nodi
    if i ~= n_nodi
        B(i, i) = 1;  % Set the diagonal elements to 1
    end
    if i > 1
        B(i, i - 1) = -1;  % Set the (i-1)th element of the ith row to -1
    end
end

kda = 25*ones(n_nodi,1);
kdb = 25*ones(n_nodi,1);

T = zeros(n_nodi -1, n_nodi );  % Initialize a matrix B of size n_nodi x (n_nodi - 1)

for i = 1:n_nodi-1
        T(i, i) = 1;  % Set the diagonal elements to 1
        T(i, i + 1) = -1;  % Set the (i-1)th element of the ith row to -1
end
