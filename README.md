# Line Follower Digital Twin Simulation

This project is a distributed simulation of a differential drive line-follower robot built on the Siemens Virtual System Interconnect (VSI) framework. It splits the system into three distinct clients that communicate over a simulated network.

## Demo Video



https://github.com/user-attachments/assets/adc3870e-9654-4b52-8203-b4dc8ec7482d



## System Components

- **Client 1: Plant Simulator (`robot_model.py`)** Simulates the physical environment. It computes the differential drive kinematics, handles the reference path (straight or Bezier curves), calculates real-time lateral and heading errors, and stops the simulation when the target is reached.
- **Client 2: PID Controller (`controller.py`)** The brain of the operation. It receives the error signals from Client 1, processes them through a PID control loop, and calculates the steering corrections needed to keep the robot on the path.
- **Client 3: Real-Time Plotter (`plotter.py`)** The monitoring dashboard. It intercepts the error data and uses `pyqtgraph` to render a live plot of the tracking performance. When the simulation ends, it calculates and prints standard control KPIs (Overshoot, Settling Time, Steady-State Error).

* * *

**Setup Note:** To run this, you generate the base clients using `vsiBuild -f vsiBuildCommands`, then inject the provided `*_stub.py` code into the respective auto-generated VSI skeleton files.
