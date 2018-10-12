
% Paul Balzer | Motorblog
% CC-BY-SA2.0 Lizenz
close all
  
%% Messungen generieren
% Da wir jetzt keine Messwerte haben,
% generieren wir uns einfach selbst
% ein paar verrauschte Werte.
it=100;     % Anzahl Messwerte
realv=10;   % wahre Geschwindigkeit
 
                   % x'                y'
measurements = [realv+1.*randn(1,it); zeros(1,it)];
erg = measurements(2,:);
  
dt = 1;     % Zeitschritt
  
%% Initialisieren
%    x  y  x' y'
x = [0; 0; 10; 0];      % Initial State (Location and velocity)
P = [10, 0, 0, 0;...
    0, 10, 0, 0;...
    0, 0, 10, 0;...
    0, 0, 0, 10];       % Initial Uncertainty
A = [1, 0, dt, 0;...
    0, 1, 0, dt;...
    0, 0, 1, 0;...
    0, 0, 0, 1];        % Transition Matrix
H = [0, 0, 1, 0;...
    0, 0, 0, 1];        % Measurement function
R = [10, 0;...
    0, 10];             % measurement noise covariance
Q = [1/4*dt^4, 1/4*dt^4, 1/2*dt^3, 1/2*dt^3;...
    1/4*dt^4, 1/4*dt^4, 1/2*dt^3, 1/2*dt^3;...
    1/2*dt^3, 1/2*dt^3, dt^2, dt^2;...
    1/2*dt^3, 1/2*dt^3, dt^2, dt^2]; % Process Noise Covariance
I = eye(4);             % Identity matrix

figure

%% Kalman Filter Steps
%
for n=1:length(measurements)
    % Prediction
    x=A*x;                  % Prädizierter Zustand aus Bisherigem und System
    P=A*P*A'+Q;             % Prädizieren der Kovarianz
  
    % Correction
    Z=measurements(:,n);
    y=Z-(H*x);              % Innovation aus Messwertdifferenz
    S=(H*P*H'+R);           % Innovationskovarianz
    K=P*H'*inv(S);          % Filter-Matrix (Kalman-Gain)
  
    x=x+(K*y);              % aktualisieren des Systemzustands
    P=(I-(K*H))*P;          % aktualisieren der Kovarianz
    
    vMeas(n) = Z(1);
    vEstim(n) = x(3);
    var(n) = P(3,3);
end

plot(1:100,vMeas,1:100,vEstim,1:100,var,1:100,X(:,3));