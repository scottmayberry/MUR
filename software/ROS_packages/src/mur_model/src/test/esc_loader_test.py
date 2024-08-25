import yaml
from sympy import symbols, lambdify, Piecewise

# Load the YAML and extract the formula
with open('./config/model_info.yaml', 'r') as file:
    data = yaml.safe_load(file)

formula_data = data['model_info']['escs']['bluerobotics']

# Define the variable
x = symbols('x')

# Generate the lambda function for the formula
formula = lambdify(x, formula_data['formula'], modules=["numpy", "sympy"])

# Define a function to evaluate the PWM value
def get_pwm_value(unit_scaled_signal):
    if abs(unit_scaled_signal) < formula_data['zero_threshold']:
        return 1500
    else:
        return round(formula(unit_scaled_signal))

# Use the function
unit_scaled_signal = 0.5  # Example value
pwm_value = get_pwm_value(unit_scaled_signal)
print(pwm_value)