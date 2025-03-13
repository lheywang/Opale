import serial
from tqdm import tqdm
import csv

# Openning serial
com = serial.Serial(port="COM7", baudrate=115200)
print(com.name)

lines_nb = 200000  # Set to 5000 for a very good experience ! (~75 /s --> One full cycle is 20 seconds, lets take 30 --> 2250 mini)
lines = []

for _ in tqdm(range(lines_nb)):
    lines.append(str(com.readline().decode()).strip())
print("End of measures !")

# end of serial
com.close()

# Creating variables outputs !
north = []
south = []
east = []
west = []
servo1 = []
servo2 = []
servo3 = []
servo4 = []

north_act = 0
south_act = 0
east_act = 0
west_act = 0


# Treating data
for index, line in tqdm(enumerate(lines)):
    s_line = line.split(" ")

    # First, making sure that the SAADC is the logger output
    try:
        if s_line[2] == "SAADC:":
            if s_line[3] == "Channel":
                # Filter channel and append to lists
                match s_line[6]:
                    case "1)":
                        servo1.append(float(s_line[10][:-1]))
                        north.append(north_act)
                    case "2)":
                        servo2.append(float(s_line[10][:-1]))
                        south.append(south_act)
                    case "3)":
                        servo3.append(float(s_line[10][:-1]))
                        east.append(east_act)
                    case "4)":
                        servo4.append(float(s_line[10][:-1]))
                        west.append(west_act)

        # Extract the command value
        if s_line[2] == "Main:":
            if s_line[3] == "Configured":
                north_act = float(lines[index + 1].split(" ")[3])
                south_act = float(lines[index + 2].split(" ")[3])
                east_act = float(
                    lines[index + 3].split(" ")[4]
                )  # Theses get one more space, so one more index
                west_act = float(lines[index + 4].split(" ")[4])

    except Exception as e:
        pass
        # print(e) # Enable for debugging purposes...

# Get the first non null value
first_val = 0
for index, val in enumerate(north):
    if val != 0:
        first_val = index
        break

# Cut the lists to this size
south = south[first_val:]
north = north[first_val:]
east = east[first_val:]
west = west[first_val:]
servo1 = servo1[first_val:]
servo2 = servo2[first_val:]
servo3 = servo3[first_val:]
servo4 = servo4[first_val:]

# Zip (change lists to rows) and write to CSV
rows = zip(north, south, east, west, servo1, servo2, servo3, servo4)
with open("tools/meas.csv", "w+", newline="") as csvfile:
    writter = csv.writer(csvfile)

    writter.writerow(
        ["north", "south", "east", "west", "servo1", "servo2", "servo3", "servo4"]
    )
    for row in rows:
        writter.writerow(row)

print("Measures treated and exported as CSV !")

# End of the script !
