import matplotlib.pyplot as plt

####
#This code parses all the .txt files to get the Webots sensor output data
#and save it as a png, which is already done. 
#This code also uses the parsed files to get the Webots position output data
#and save it as a png, which is already done.
###

def plotdata(data_arr, name):
    if name[0] == "S":
        factor = 10.0
        title = "Segway Webot Simulation Sensor Data ({})".format(name)
        title2 = "Segway Webot Simulation Trajectory ({})".format(name)
    else:
        factor = 1.0
        title = "Paperbot Webot Simulation Sensor Data ({})".format(name)
        title2 = "Paperbot Webot Simulation Trajectory ({})".format(name)
    xcompass = [float(x[2]) for x in data_arr]
    ycompass = [float(x[4]) for x in data_arr]
    in_plane_gyro = [float(x[6]) for x in data_arr]
    lidar_front = [(float(x[8])/4096.0)*factor for x in data_arr]
    lidar_right = [(float(x[9])/4096.0)*factor for x in data_arr]
    x_pos = [float(x[10]) for x in data_arr]
    y_pos = [float(x[12]) for x in data_arr]

    fig = plt.figure()
    plt.plot(xcompass, color='red', label='Compass (X)')
    plt.plot(ycompass, color='blue', label='Compass (Y)')
    plt.plot(in_plane_gyro, color='green', label='In Plane Gyro')
    plt.plot(lidar_front, color='orange', label='Front Lidar')
    plt.plot(lidar_right, color='purple', label='Right Lidar')
    plt.xlabel('Time (ms)')
    plt.ylabel('Sensor Reading (scaled to world)')
    plt.title(title)
    plt.legend()
    fig.savefig('{}_sensor.png'.format(name), dpi=fig.dpi)

    fig2 = plt.figure()
    plt.plot(x_pos, y_pos)
    plt.title(title2)
    plt.xlabel("x (m) from origin")
    plt.ylabel("y (m) from origin")
    fig2.savefig('{}_webot_trajectory.png'.format(name), dpi=fig2.dpi)


gmp1data = []
with open("./griffin/POutput1GM.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        gmp1data.append(line[:-1].split(","))
plotdata(gmp1data, "POutput1GM")

gmp2data = []
with open("./griffin/POutput2GM.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        gmp2data.append(line[:-1].split(","))
plotdata(gmp2data, "POutput2GM")

gmp3data = []
with open("./griffin/POutput3GM.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        gmp3data.append(line[:-1].split(","))
plotdata(gmp3data, "POutput3GM")

gmp4data = []
with open("./griffin/POutput4GM.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        gmp4data.append(line[:-1].split(","))
plotdata(gmp4data, "POutput4GM")

gmp5data = []
with open("./griffin/POutput5GM.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        gmp5data.append(line[:-1].split(","))
plotdata(gmp5data, "POutput5GM")

gmp6data = []
with open("./griffin/POutput6GM.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        gmp6data.append(line[:-1].split(","))
plotdata(gmp6data, "POutput6GM")

gms1data = []
with open("./griffin/SOutput1GM.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        gms1data.append(line[:-1].split(","))
plotdata(gms1data, "SOutput1GM")

gms2data = []
with open("./griffin/SOutput2GM.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        gms2data.append(line[:-1].split(","))
plotdata(gms2data, "SOutput2GM")

gms3data = []
with open("./griffin/SOutput3GM.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        gms3data.append(line[:-1].split(","))
plotdata(gms3data, "SOutput3GM")

gms4data = []
with open("./griffin/SOutput4GM.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        gms4data.append(line[:-1].split(","))
plotdata(gms4data, "SOutput4GM")

gms5data = []
with open("./griffin/SOutput5GM.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        gms5data.append(line[:-1].split(","))
plotdata(gms5data, "SOutput5GM")

gms6data = []
with open("./griffin/SOutput6GM.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        gms6data.append(line[:-1].split(","))
plotdata(gms6data, "SOutput6GM")

#####

smp1data = []
with open("./sean/POutput1SM.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        smp1data.append(line[:-1].split(","))
plotdata(smp1data, "POutput1SM")

smp2data = []
with open("./sean/POutput2SM.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        smp2data.append(line[:-1].split(","))
plotdata(smp2data, "POutput2SM")

smp3data = []
with open("./sean/POutput3SM.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        smp3data.append(line[:-1].split(","))
plotdata(smp3data, "POutput3SM")

smp4data = []
with open("./sean/POutput4SM.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        smp4data.append(line[:-1].split(","))
plotdata(smp4data, "POutput4SM")

smp5data = []
with open("./sean/POutput5SM.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        smp5data.append(line[:-1].split(","))
plotdata(smp5data, "POutput5SM")

smp6data = []
with open("./sean/POutput6SM.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        smp6data.append(line[:-1].split(","))
plotdata(smp6data, "POutput6SM")

sms1data = []
with open("./sean/SOutput1SM.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        sms1data.append(line[:-1].split(","))
plotdata(sms1data, "SOutput1SM")

sms2data = []
with open("./sean/SOutput2SM.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        sms2data.append(line[:-1].split(","))
plotdata(sms2data, "SOutput2SM")

sms3data = []
with open("./sean/SOutput3SM.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        sms3data.append(line[:-1].split(","))
plotdata(sms3data, "SOutput3SM")

sms4data = []
with open("./sean/SOutput4SM.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        sms4data.append(line[:-1].split(","))
plotdata(sms4data, "SOutput4SM")

sms5data = []
with open("./sean/SOutput5SM.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        sms5data.append(line[:-1].split(","))
plotdata(sms5data, "SOutput5SM")

sms6data = []
with open("./sean/SOutput6SM.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        sms6data.append(line[:-1].split(","))
plotdata(sms6data, "SOutput6SM")

#####

arp1data = []
with open("./andrea/POutput1AR.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        arp1data.append(line[:-1].split(","))
plotdata(arp1data, "POutput1AR")

arp2data = []
with open("./andrea/POutput2AR.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        arp2data.append(line[:-1].split(","))
plotdata(arp2data, "POutput2AR")

arp3data = []
with open("./andrea/POutput3AR.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        arp3data.append(line[:-1].split(","))
plotdata(arp3data, "POutput3AR")

arp4data = []
with open("./andrea/POutput4AR.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        arp4data.append(line[:-1].split(","))
plotdata(arp4data, "POutput4AR")

arp5data = []
with open("./andrea/POutput5AR.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        arp5data.append(line[:-1].split(","))
plotdata(arp5data, "POutput5AR")

arp6data = []
with open("./andrea/POutput6AR.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        arp6data.append(line[:-1].split(","))
plotdata(arp6data, "POutput6AR")

ars1data = []
with open("./andrea/SOutput1AR.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        ars1data.append(line[:-1].split(","))
plotdata(ars1data, "SOutput1AR")

ars2data = []
with open("./andrea/SOutput2AR.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        ars2data.append(line[:-1].split(","))
plotdata(ars2data, "SOutput2AR")

ars3data = []
with open("./andrea/SOutput3AR.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        ars3data.append(line[:-1].split(","))
plotdata(ars3data, "SOutput3AR")

ars4data = []
with open("./andrea/SOutput4AR.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        ars4data.append(line[:-1].split(","))
plotdata(ars4data, "SOutput4AR")

ars5data = []
with open("./andrea/SOutput5AR.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        ars5data.append(line[:-1].split(","))
plotdata(ars5data, "SOutput5AR")

ars6data = []
with open("./andrea/SOutput6AR.txt", "r") as filestream:
    next(filestream)
    for line in filestream:
        ars6data.append(line[:-1].split(","))
plotdata(ars6data, "SOutput6AR")
