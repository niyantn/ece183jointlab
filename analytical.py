import matplotlib.pyplot as plt
import numpy as np
import math
import random
import pandas as pd

###
#This code parses the files for trajectories and runs them on
#the ECE analytical simulation
###

# grid formulation (100x100)
x_max = 100
y_max = 100
# north vector
north = [0, 1]
# delta t
delta_t = 0.01


# robot formulation
class Robot:
    def __init__(self, xpos, ypos, theta, wheel_d):
        # angles in radians
        self.state = [xpos, ypos,
                      theta, wheel_d]  # distance from front wall, distance from right wall, orientation as degree value where pi/2 is straight

    def pwmToRotVel(self, input_):
        output = np.array(100 * np.tanh(input_))
        # https://www.geeksforgeeks.org/numpy-tanh-python/ i used np beacuse of this, couldve been wrong though
        # output[0] = 100 * np.tanh(2 * r_vx)
        # output[1] = 100 * np.tanh(2 * r_vy)
        return output

    def updateState(self, nextState):
        if nextState[0] > 0 and nextState[0] < 100:
            self.state[0] = nextState[0]
        if nextState[1] > 0 and nextState[1] < 100:
            self.state[1] = nextState[1]
        if nextState[2] > 0 and nextState[2] < 100:
            self.state[2] = nextState[2]

    def getNextState(self, input_):
        # convert PWM to rotational velocity
        # rot_vel =[0, 0 #placeholder?
        # rot_vel = np.zeros(2) does it need to be instantiated first?
        #rot_vel = self.pwmToRotVel(input_) COMMENTED THIS OUT FOR LAB 2: now, can directly input angular velocities
        rot_vel = np.array(input_)
        # rot_vel[1] = self.pwmToRotVel(input_[1])

        # convert rotational vel to velocity
        # velocity = [0,0] #placeholder?
        # velocity = np.zeros(2)
        # velocity[0] = rot_vel[0] * (wheel_d / 2)
        # velocity[1] = rot_vel[1] * (wheel_d / 2)
        velocity = rot_vel * (self.state[3] / 2)

        vbar = (velocity[0] + velocity[1]) / 2

        # delta x1, x2, x3
        delta_x = vbar * np.cos(self.state[2])
        delta_y = vbar * np.sin(self.state[2])
        omega = velocity[0] - velocity[1]

        nState = [0, 0, 0, 0]  # placeholder?

        nState[0] = self.state[0] + (delta_x * delta_t)
        nState[1] = self.state[1] + (delta_y * delta_t)
        nState[2] = self.state[2] + (omega * delta_t)
        nState[3] = self.state[3]

        return nState

    def state_dynamic_equation(self, input_, noise=None, time=None):
        # update state (member variables) from input_, noise, and time
        if len(input_) != 2:
            return "Please enter a two element input_! [right wheel PWM, left wheel PWM]"
        if noise:
            return "Haven't implemented noise yet, sorry! Please try again."
        if time:
            return "Ignoring time due to Markov property! Please try again."

        # get next state
        nextState = self.getNextState(input_)

        # Update State
        self.updateState(nextState)

    def output_equation(self, input_, noise=None, time=None):
        # return output as 5 dimensional vector from state (member variables), input_, noise, and time
        if len(input_) != 2:
            return "Please enter a two element input_! [right wheel PWM, left wheel PWM]"
        if noise:
            return "Haven't implemented noise yet, sorry! Please try again."
        if time:
            return "Ignoring time due to Markov property! Please try again."

        output_vec = [0] * 5

        x = self.state[0]
        y = self.state[1]
        angle = self.state[2]

        def getMainLineIntersection(x, y, angle):
            while angle >= 2 * np.pi:
                angle -= 2 * np.pi
            while angle < 0:
                angle += 2 * np.pi
            if angle == 0:
                xwf = x_max
                ywf = y
            elif angle == np.pi / 2:
                xwf = x
                ywf = y_max
            elif angle == np.pi:
                xwf = 0
                ywf = y
            elif angle == 3 * np.pi / 2:
                xwf = x
                ywf = 0
            else:
                slope = np.tan(angle)
                intercept = y - (slope * x)
                ywf = min(y_max, slope * x_max + intercept)
                ywf = max(ywf, 0)
                # ywb = slope*0 + intercept
                xwf = min(x_max, (y_max - intercept) / slope)
                xwf = max(xwf, 0)
                # xwb = (0 - intercept) / slope
            return (xwf, ywf)

        def getPerpLineIntersection(x, y, angle):
            angle -= np.pi / 2
            while angle >= 2 * np.pi:
                angle -= 2 * np.pi
            while angle < 0:
                angle += 2 * np.pi
            if angle == 0:
                xwr = x_max
                ywr = y
            elif angle == np.pi / 2:
                xwr = x
                ywr = y_max
            elif angle == np.pi:
                xwr = 0
                ywr = y
            elif angle == 3 * np.pi / 2:
                xwr = x
                ywr = 0
            else:
                slope = np.tan(angle)
                intercept = y - (slope * x)
                ywr = min(y_max, slope * x_max + intercept)
                ywr = max(ywr, 0)
                # ywl = slope*0 + intercept
                # xwl = (y_max - intercept) / slope
                xwr = min(x_max, (0 - intercept) / slope)
                xwr = max(xwr, 0)
            return (xwr, ywr)

        xwf, ywf = getMainLineIntersection(x, y, angle)
        xwr, ywr = getPerpLineIntersection(x, y, angle)
        output_vec[0] = np.sqrt(
            (xwf - self.state[0]) ** 2 + (ywf - self.state[1]) ** 2)  # distance to the wall in front
        output_vec[1] = np.sqrt(
            (xwr - self.state[0]) ** 2 + (ywr - self.state[1]) ** 2)  # distance to the wall to the right

        # output_vec[0] = np.linalg.norm(np.array([xwf,ywf]) - np.array([self.state[0],self.state[1]]))
        # output_vec[1] = np.linalg.norm(np.array([xwr,ywr]) - np.array([self.state[0],self.state[1]]))
        # convert PWM to rotational velocity
        rot_vel = self.pwmToRotVel(input_)
        velocity = rot_vel * (self.state[3] / 2)
        omega = velocity[0] - velocity[1]
        output_vec[2] = omega  # in plane rotational speed

        # take dot product of position vector with north, divide by their magnitudes, and take inverse cosine to get angle phi
        phi = np.arccos(
            (self.state[0] * north[0] + self.state[1] * north[1]) / np.linalg.norm([self.state[0], self.state[1]]))
        output_vec[3] = np.cos(phi)  # magnetic field in x direction
        output_vec[4] = np.sin(phi)  # magnetic field in y direction

        return output_vec

def plotdata(data_arr, name):
    if name[0] == "S":
        wheel_d = 5
        title = "Segway Analytical Simulation Trajectory ({})".format(name)
    else:
        wheel_d = .5
        title = "Paperbot Analytical Simulation Trajectory ({})".format(name)
    leftvels = [float(x[0]) for x in data_arr]
    rightvels = [float(x[1]) for x in data_arr]
    n = len(leftvels)
    xposes = []
    yposes = []
    rob = Robot(50, 50, 0, wheel_d)
    for i in range(n):
        rob.state_dynamic_equation([leftvels[i]/10, rightvels[i]/10])
        xposes.append(rob.state[0])
        yposes.append(rob.state[1])

    x_pos = [float(x[10]) for x in data_arr]
    y_pos = [float(x[12]) for x in data_arr]

    percent_diff_x = [((xposes[i]/10)-x_pos[i])/xposes[i] for i in range(len(xposes))]
    percent_diff_y = [((yposes[i]/10)-y_pos[i])/yposes[i] for i in range(len(yposes))]
    with open("percent_diffs.txt", "a") as filestream:
        filestream.write(str((sum(percent_diff_x)/len(percent_diff_x), sum(percent_diff_y)/len(percent_diff_y))))
        filestream.write("\n")
    fig = plt.figure()
    plt.plot(xposes, yposes)
    plt.xlabel('x (m) from origin (50,50)')
    plt.ylabel('y (m) from origin (50,50)')
    plt.title(title)
    fig.savefig('{}_analytical_trajectory.png'.format(name), dpi=fig.dpi)

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