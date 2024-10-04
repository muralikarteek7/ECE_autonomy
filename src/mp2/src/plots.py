import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
csv_file = 'vehicle_data.csv'
data = pd.read_csv(csv_file)

# Check if the data is loaded
if data.empty:
    print("No data found in the CSV file.")
else:
    # Plot x vs y
    plt.figure(figsize=(12, 5))

    plt.subplot(1, 2, 1)  # 1 row, 2 columns, 1st subplot
    plt.plot(data['x'], data['y'], marker='o', linestyle='-', color='b')
    plt.title('X vs Y')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid()

    # Plot acceleration vs time
    plt.subplot(1, 2, 2)  # 1 row, 2 columns, 2nd subplot
    plt.plot(data['time'], data['acceleration'], marker='o', linestyle='-', color='r')
    plt.title('Acceleration vs Time')
    plt.xlabel('Time')
    plt.ylabel('Acceleration')
    plt.grid()

    # Show the plots
    plt.tight_layout()
    plt.show()
