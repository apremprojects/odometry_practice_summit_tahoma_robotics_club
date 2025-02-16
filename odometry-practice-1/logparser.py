import matplotlib.pyplot as plt
from argparse import ArgumentParser
import re

def string_to_float(s):
    numbers = re.findall(r"\d+\.?\d*", s)
    return float(''.join(numbers))

if __name__ == "__main__":
    #plt.figure(figsize=(4, 3))
    parser = ArgumentParser(
        prog='LogParser',
        add_help=False
    )
    parser.add_argument('filename')
    parser.add_argument('--tmax', type = float, default = -1)
    args = parser.parse_args()
    

    ax = plt.figure().add_subplot(111) #to help annotate

    if args.filename == None:
        raise Error("No file provided")
    else:
        f = open(args.filename, "r")
        
        pos_x = []
        pos_y = []
        pos_time = []
        targ_pos_x = []
        targ_pos_y = []
        targ_pos_time = []
        for line in f:
            split_line = line.split()
            timestamp = split_line[0]
            t = int(timestamp.split(":")[2]) + (int(timestamp.split(":")[1]) * 60)
            if args.tmax != -1:
                if t > args.tmax:
                    break
            message_type = split_line[1]


            if message_type == "POS_UPDATE":
                pos_x.append(string_to_float(split_line[2]))
                pos_y.append(string_to_float(split_line[3]))
                pos_time.append(timestamp)
            elif message_type == "TARGET_POS_UPDATE":
                targ_pos_x.append(string_to_float(split_line[2]))
                targ_pos_y.append(string_to_float(split_line[3]))
                targ_pos_time.append(timestamp)
        targ_pos_x.insert(0, pos_x[0])
        targ_pos_y.insert(0, pos_y[0])
        plt.scatter(pos_x, pos_y, alpha = 1, label = "Pos", color = "blue")
        plt.scatter(targ_pos_x, targ_pos_y, alpha = 1, label = "Target Pos", color = "red")
        for i in range(0, len(pos_time), 50):
            t = int(pos_time[i].split(":")[2]) + (int(pos_time[i].split(":")[1]) * 60)
            ax.annotate(t, xy=(pos_x[i], pos_y[i]), xytext=(0,0), textcoords='offset points')
        print(len(pos_x))
        print(len(targ_pos_x))
        plt.gca().set_xlim([0, 3600])
        plt.gca().set_ylim([0, 3600])
        #plt.show()

        f = open(args.filename, "r")
        target_angles = []
        target_angle_times = []
        actual_angles = []
        actual_angle_times = []
        for line in f:
            split_line = line.split()
            timestamp = split_line[0]
            message_type = split_line[1]
            t = (float(timestamp.split(":")[3]) / 1000.0) + float(timestamp.split(":")[2]) + (float(timestamp.split(":")[1]) * 60)
            if message_type == "ANGLE_UPDATE":
                actual_angles.append(float(split_line[2]))
                actual_angle_times.append(t)
            elif message_type == "TARGET_ANGLE_UPDATE":
                target_angles.append(float(split_line[2]))
                target_angle_times.append(t)
    print(len(actual_angles))
    print(len(target_angles))
    print(target_angle_times)
    target_angles_draw = []
    target_angle_times_draw = []
    for i in range(0, len(target_angles), 50):
        target_angles_draw.append(target_angles[i])
        target_angle_times_draw.append(target_angle_times[i])

    actual_angles_draw = []
    actual_angle_times_draw = []
    for i in range(0, len(actual_angles), 50):
        actual_angles_draw.append(actual_angles[i])
        actual_angle_times_draw.append(actual_angle_times[i])
    plt.figure()
    plt.plot(target_angle_times_draw, target_angles_draw, alpha=1, label="target angles", color = "blue")
    plt.plot(actual_angle_times_draw, actual_angles_draw, alpha=1, label="actual angles", color = "red")
    plt.gca().set_ylim([-20, 20])
    plt.show()