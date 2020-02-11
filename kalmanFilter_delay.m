function [yhatOut] = kalmanFilter(u,meas)
% This Embedded MATLAB Function implements a very simple Kalman filter.
%
% It implements a Kalman filter for estimating both the state and output
% of a linear, discrete-time, time-invariant, system given by the following
% state-space equations:
%
% x(k) = 0.914 x(k-1) + 0.25 u(k) + w(k)
% y(k) = 0.344 x(k-1) + v(k)
%
% where w(k) has a variance of 0.01 and v(k) has a variance of 0.1.

% Author: Phil Goddard (phil@goddardconsulting.ca)
% Date: Q2, 2011.

% Define storage for the variables that need to persist
% between time periods.
%R = R*1.5;
persistent P xhat A B C Q R
if isempty(P)
% First time through the code so do some initialization
   xhat = 0;
   P = 0;
   A = 0.914;   % already patent
   B = 0.25;    % already patent
   C = 0.344;   % already patent
   Q = 0.5;   % for good results of noisy signal: 0.01^2;
                % for resembling measurement results: 0.025;
   R = 0.1;  % for good results of noisy signal: 0.1^2;
                % for resembling measurement results: 0.0001;
   
   % best parameters to date:
   %    P = 0;
   %    A = 0.914;
   %    B = 0.25;
   %    C = 0.344;
   %    Q = 0.01^2;
   %    R = 0.1;
end
% Propagate the state estimate and covariance matrix:
xhat = A*xhat + B*u;
P = A*P*A' + Q;
% Calculate the Kalman gain
K = P*C'/(C*P*C' + R);
% Calculate the measurement residual
resid = meas - C*xhat;
% Update the state and error covariance estimate
xhat = xhat + K*resid;
P = (eye(size(K,1))-K*C)*P;
% Post the results
xhatOut = xhat;
yhatOut = C*xhatOut;