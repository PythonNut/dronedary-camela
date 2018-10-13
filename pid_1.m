% coefficients for PID
Kp = 1;
Ki = 1;
Kd = 1;

% simulate transfer function
s = tf('s');
C = Kp + Ki/s + Kd*s;
step(C);