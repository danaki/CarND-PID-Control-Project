### PID

PID states for Proportional-Integral-Derivative, the three components of the algorithm, where Proportional adjusts vehicle position proportionaly to the CTE (cross-track error), Derivative part of the equations takes into account how fast CTE changes during update states and Integral part allows to fight the systematic error by accumulating it over several update steps.

### Hyperparameters

Hyperparameters (Kp, Kd and Ki) were chosen by running the twiddle algorithm after it coverged after several hundreds of trials.

### Video

https://youtu.be/iXYeivC94kQ
