from Phidget22.Phidget import *
from Phidget22.Devices.VoltageRatioInput import *
import time

# Define calibration factor (determined through calibration)
CALIBRATION_FACTOR = 4.6596e+007  # Adjust this based on your calibration process

# Global variable to store the first captured value (TARE)
tare_value = None

# Event handler for voltage ratio change
def onVoltageRatioChange(self, voltageRatio):
    global tare_value

    # Capture the first reading as the TARE value
    if tare_value is None:
        tare_value = voltageRatio
        print(f"TARE Value Set: {tare_value:.6f}")

    # Apply TARE: Adjust voltage ratio by subtracting the initial value
    adjusted_ratio = voltageRatio - tare_value

    # Convert to weight
    weight = adjusted_ratio * CALIBRATION_FACTOR

    print(f"VoltageRatio (TARED): {adjusted_ratio:.6f}, Estimated Weight: {weight:.2f} gm")

def main():
    # Create Phidget channel
    voltageRatioInput = VoltageRatioInput()
    
    try:
        # Assign event handler
        voltageRatioInput.setOnVoltageRatioChangeHandler(onVoltageRatioChange)

        # Open Phidget and wait for attachment
        voltageRatioInput.openWaitForAttachment(5000)
        voltageRatioInput.setDataInterval(25)

        print("Reading data... Press Ctrl+C to stop.")

        # Keep script running
        while True:
            time.sleep(0.025)

    except PhidgetException as e:
        print(f"Phidget Error: {e.details}")

    except KeyboardInterrupt:
        print("\nUser interrupted. Exiting...")

    finally:
        voltageRatioInput.close()

if __name__ == "__main__":
    main()
