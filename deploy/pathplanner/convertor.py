import csv
import math
import pyperclip

path_name = input("Enter the name of the path you want to convert: ")

reader = csv.reader(open('deploy/pathplanner/generatedCSV/' + path_name + '.csv'))
reader.__next__()
reader.__next__()
path = list(reader)

x = 0.0
y = 0.0
prev_time = 0.0

out = 'ryan::TimedTrajectory Paths::' + path_name + '{\n'

for row in path:
    # x_vel = (float(row[2]) * math.cos(math.radians(float(row[8]))) * 3.28084) / (float(row[0]) - prev_time + 0.000000001)
    # y_vel = (float(row[2]) * math.sin(math.radians(float(row[8]))) * 3.28084) / (float(row[0]) - prev_time + 0.000000001)
    x_vel = (float(row[4]) * math.cos(math.radians(float(row[8]))))
    y_vel = (float(row[4]) * math.sin(math.radians(float(row[8]))))

    if(path.index(row) < len(path)-1):
        x += x_vel * (float(path[path.index(row)+1][0]) - float(row[0]))
        y += y_vel * (float(path[path.index(row)+1][0]) - float(row[0]))
    # prev_time = float(row[0])

    # x += x_vel * (float(row[0]) - prev_time)
    # y += y_vel * (float(row[0]) - prev_time)

    out += '{' + str(row[0]) + ', ' + str(row[1]) + ', ' + str(row[2]) + ', ' + str(row[3]) + ', ' + str(row[4]) + ', ' + str(row[8])

    if(path.index(row) == len(path)-1):
        out += '}\n'
    else:
        out += '},\n'

out += "};"

pyperclip.copy(out)

print("Success!! Path copied to clipboard!")
