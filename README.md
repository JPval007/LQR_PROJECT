# LQR_PROJECT

For this project the main goal was to design and implement a controller for a Buck converter, using modern modeling and control techniques.
This project was divided in three phases:

1. Modeling:
The system model was created by using a state space model with the capacitor voltage as a state variable and an observer to estimate the inductor current.

2. Design and simulations of the controller:
The model was simulated using Matlab and Simscape using the Simscape Electrical library. 

3. Physical system and TivaC LQR code:
Finally, it was implemented in the TivaC board to stabilize the system's response to sudden changes to the input voltage and frecuency of the PWM signal.
