import numpy as np
import matplotlib.pyplot as plt

ZERO_THRESHOLD_FOR_THRUSTERS = 0.01  # Adjust this value as needed

def unit_scaled_thruster_input_to_pwm_signal(unit_scaled_signal):
    if abs(unit_scaled_signal) < ZERO_THRESHOLD_FOR_THRUSTERS:
        return 1500
    else:
        calc = 3.5 * abs(unit_scaled_signal)
        if unit_scaled_signal > 0:
            out = round(-10.8 * calc**2 + calc * 119.5 + 1540)
        else:
            out = round(11.31 * calc**2 + calc * -137.4 + 1456)
        return out

# Generate data
x = np.linspace(-1, 1, 400)
y = [unit_scaled_thruster_input_to_pwm_signal(val) for val in x]

# Plot
plt.figure(figsize=(10, 6))
plt.plot(x, y, label='PWM Signal', color='blue')
plt.axhline(y=1500, color='r', linestyle='--', label='1500 μs')
plt.title('PWM Signal vs. Unit Scaled Thruster Input')
plt.xlabel('Unit Scaled Thruster Input')
plt.ylabel('PWM Signal (μs)')
plt.legend()
plt.grid(True)
plt.show()