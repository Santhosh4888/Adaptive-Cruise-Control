# Autonomous Driving Workspace with Simulated obstacle

This is the workspace used without sensors. i.e Using a simulated obstacle as the lead object.

In this workspace both PD and MPC Controller are tested using simulated obstacle with test conditions as :
    1. Object is detected after the vehicle has moved 10m i.e at 90m and controller starts taking action in ACC mode
    2. The MPC controller weigths are unity(1) with CBF as hard constraint and few other reduncdant constraints.
    3. The stopping distance for MPC is very high around 4-7m with crawling behaviour.
