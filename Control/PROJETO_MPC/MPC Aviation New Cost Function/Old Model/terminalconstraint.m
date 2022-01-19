%
% This script generates the terminal set to the IMFR
%
% Author: Pedro Assis
% Date: 29/10/16
%
function [Amf, Smas] = terminalconstraint(A, B, K, Ts, nx, umax, umin, xmax, xmin)
Amf = (A - B*K);

sys = LTISystem('A', Amf,'Ts',Ts);

H = [-K;
       K;
       eye(nx);
      -eye(nx)];

b = [ umax;
      -umin;
       xmax;
      -xmin];

% Generates the polyhedron formed by the constraints
X = Polyhedron('H',[H b]);

% Determining the invariant set
Smas = sys.invariantSet('X', X); % S.A \leq S.b
