import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from scipy.interpolate import PchipInterpolator

# Given data points
desired_force = np.array([2.15, 2.2, 2.275, 2.4, 6.5, 9.5, 11])
enc_angle = np.array([0, 10, 20, 40, 70, 80, 85])  # Convert degrees to radians

# Create the PCHIP interpolator
pchip_fit = PchipInterpolator(enc_angle, desired_force)



# Load the CSV file into a pandas DataFrame
file_path = 'phidgetData.csv'  # Update with your CSV file path
data = pd.read_csv(file_path)
print(data.shape)

# plt.plot(data['weight'].values)
# plt.show()
# Define the index range (adjust these values based on your requirements)
start_index = 300  # Starting index (inclusive)
end_index = 1500    # Ending index (exclusive)

# start_index = 290  # Starting index (inclusive)
# end_index = 2620   # Ending index (exclusive)

angle = vector = np.linspace(5, 65, end_index-start_index)
# Generate values for interpolation (for smooth curve)
force_fine = pchip_fit(angle)

# Filter data to include only rows within the specified index range
filtered_data = data.iloc[start_index:end_index]

# Extract the filtered x and y values
angle_filt = [5,10,15,20,25,30,35,40,45,50,55,60,65,66,67,68,69,70]
# force_filtered = [x * 10 / 1000 for x in [157, 194, 214, 224, 231, 234, 234, 243, 279, 302, 317, 399, 465, 525, 583,605, 632, 670]]
force_filtered = [x * 9.81 / 1000 for x in [154, 210, 231, 242, 250, 255, 263, 263, 297, 327, 359, 402, 499, 546, 598, 610, 632, 670]]


# Fit a polynomial of degree n (adjust n as needed)
# degree = 5  # Polynomial degree, for example, quadratic (2)
# coefficients = np.polyfit(angle_filt, force_filtered, degree)

# Generate a polynomial function from the coefficients
# polynomial = np.poly1d(coefficients)

# Generate values for plotting the fitted curve
x_fit = np.linspace(5, 65, 100)  # Range between x_min and x_max
# y_fit = polynomial(x_fit)

# Plotting the original data and the fitted polynomial curve
plt.plot(angle_filt, force_filtered, color='red', marker='o', label='Force Experimental')
# plt.plot(x_fit, y_fit, label=f'Polynomial Fit (degree={degree})', color='blue')
plt.plot(angle,force_fine,linestyle="--", label='Force Desired')
plt.xlabel('angle')
plt.ylabel('force')
plt.legend()
plt.grid(visible=True)
plt.title(f'Polynomial Fit to Data (x between {5} and {65})')
plt.show()

# Print the polynomial coefficients
# print("Polynomial coefficients:", coefficients)
