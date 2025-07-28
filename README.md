# SRSSUKF - Square-Root Spherical Simplex Unscented Kalman Filter (MATLAB)

This repository provides a MATLAB implementation of the **Square-Root Spherical Simplex Unscented Kalman Filter (SRSSUKF)** for nonlinear state estimation problems.
It is suitable for systems with nonlinear process and/or measurement models.
The implementation supports the state vector containing one or more angular quantities, including unit quaternions and angles, with appropriate handling of their special properties.


## Features

- Lightweight: single-file implementation (`SRSSUKF.m`)
- Callbacks: Easily plug in custom process and measurement models via user-defined function handles
- Support for quaternion and angle state variables
- Sequential update support for multi-sensor fusion


## Usage

The entire implementation is contained in a single MATLAB class file: `SRSSUKF.m`.
To use the filter, simply ensure this file is on your MATLAB path.
No additional dependencies or setup steps are required.
To use this filter:
1. **Construct the filter** using the `SRSSUKF` constructor (see `SRSSUKF.SRSSUKF`).
2. **Initialize the filter** by calling the `Initialize` method with the initial state and square-root covariance matrix.
3. **Perform time updates** using the `Predict` method.
4. **Incorporate measurements** using the `Update` method. This method can be called multiple times in sequence (sequential sensor fusion).


## Documentation

- Full API documentation is available in the MATLAB class file: `SRSSUKF.m`
- Key methods: `Initialize`, `Predict`, `Update`


## Example (MATLAB)

The following example demonstrates how to use the SRSSUKF API with a simple linear system, serving as a minimal and self-contained illustration of the filter’s usage.

Construct a filter object
```
w0   = 0.5;                  % weight parameter for sigma-point generation
xDim = 2;                    % state dimension
wDim = 2;                    % process noise dimension (state is augmented with process noise)
idxA = [];                   % no angles in x
idxQ = [];                   % no quaternions in x
yDim = {1, 1};               % measurement dimension: two sensors with yDim1 = 1, yDim2 = 1
vDim = {1, 1};               % measurement noise dimension: two sensors with vDim1 = 1, vDim2 = 1
filter = SRSSUKF(w0, xDim, wDim, idxA, idxQ, yDim, vDim);
```

Initialize the filter with initial state and covariance
```
x0 = [0; 0];                 % initial state
P  = eye(2);                 % initial covariance for state
Q  = eye(2);                 % initial covariance for process noise
R  = eye(2);                 % initial covariance for measurement noise
Sx = chol(P,'lower');
Sw = chol(Q,'lower');
Sv = chol(R,'lower');
S0 = blkdiag(Sx, Sw, Sv);    % initial square-root covariance matrix
filter.Initialize(x0, S0);
```

Predict state and square-root covariance
```
Ts = 0.01;                  % sampletime in seconds
u  = 0;                     % input value
funcProc = @(x,w,u,Ts,optArg)([x(1) + Ts*x(2) + 0.5*Ts*Ts*u; x(2) + Ts*u]);
filter.Predict(funcProc, Ts, u, [], []);
```

Update state by applying two sensor measurements
```
y1 = 0;                     % measurement value of sensor 1
y2 = 0;                     % measurement value of sensor 2
funcObsv1 = @(x,v,optArg)(x(1));
funcObsv2 = @(x,v,optArg)(x(2));
filter.Update(funcObsv1, 1, y1, [], [], []);
filter.Update(funcObsv2, 2, y2, [], [], []);
```

Obtain current state and covariance estimation
```
x = filter.GetState();
P = filter.GetCovariance();
```

