#!/bin/python3

# Template for traffic simulation
# BH, MP 2021-11-15, latest version 2022-11-1.

"""
    This template is used as backbone for the traffic simulations.
    Its structure resembles the one of the pendulum project, that is you have:
    (a) a class containing the state of the system and it's parameters
    (b) a class storing the observables that you want then to plot
    (c) a class that propagates the state in time (which in this case is discrete), and
    (d) a class that encapsulates the aforementioned ones and performs the actual simulation
    You are asked to implement the propagation rule(s) corresponding to the traffic model(s) of the project.
"""

import math
import matplotlib.pyplot as plt
from matplotlib import animation
import numpy.random as rng
import numpy as np
import statistics
import matplotlib
font = {'weight' : 'normal',
        'size'   : 16}

plt.rc('font', **font)


class Cars:

    """ Class for the state of a number of cars """

    def __init__(self, numCars=5, roadLength=50, v0=1, lanes=2):
        self.numCars    = numCars
        self.roadLength = roadLength
        self.lanes= lanes
        self.t  = 0
        self.x  = []
        self.y = []
        self.v  = []
        self.c  = []
        for i in range(numCars):
            self.x.append(i)        # the position of the cars on the road
            self.y.append(1)
            self.v.append(v0)       # the speed of the cars
            self.c.append(i)        # the color of the cars (for drawing)

    def distance(self, i, lane):
        # TODO: Implement the function returning the PERIODIC distance 
        # between car i and the one in front 

        cars_in_lane = np.array([])
        for ii in range(self.numCars):
            if self.y[ii] == lane:
                cars_in_lane = np.append(cars_in_lane, self.x[ii])
        if cars_in_lane.size == 0:
            return 1000

        if self.x[i] >= np.max(cars_in_lane):    
            return self.roadLength - (self.x[i] - np.min(cars_in_lane))
        else:
            dist = cars_in_lane - self.x[i]
            if lane == self.y[i]:
                return np.min(dist[np.where(dist>0)])
            else: 
                return np.min(dist[np.where(dist>=0)])


class Observables:

    """ Class for storing observables """

    def __init__(self):
        self.time = []          # list to store time
        self.flowrate = []      # list to store the flow rate
        self.pos = []

class BasePropagator:

    def __init__(self):
        return
        
    def propagate(self, cars, obs):

        """ Perform a single integration step """
        
        fr = self.timestep(cars, obs)

        # Append observables to their lists
        obs.time.append(cars.t)
        obs.flowrate.append(fr)  # CHANGE!
        obs.pos.append(cars.x.copy())
              
    def timestep(self, cars, obs):

        """ Virtual method: implemented by the child classes """
        
        pass
      
        
class ConstantPropagator(BasePropagator) :
    
    """ 
        Cars do not interact: each position is just 
        updated using the corresponding velocity 
    """
    
    def timestep(self, cars, obs):
        for i in range(cars.numCars):
            cars.x[i] += cars.v[i]
        cars.t += 1
        return 0

# TODO
# HERE YOU SHOULD IMPLEMENT THE DIFFERENT CAR BEHAVIOR RULES
# Define you own class which inherits from BasePropagator (e.g. MyPropagator(BasePropagator))
# and implement timestep according to the rule described in the project

class MyPropagator(BasePropagator) :

    def __init__(self, vmax, p):
        BasePropagator.__init__(self)
        self.vmax = vmax
        self.p = p

    def timestep(self, cars, obs):
        
        print(cars.y)
        for i in range(cars.numCars):
            if cars.v[i]<self.vmax:
                cars.v[i]+=1
                      
        for i in range(cars.numCars):
            d = cars.distance(i, cars.y[i])
            if cars.y[i] == cars.lanes and cars.v[i]>=d:
                cars.v[i] = d-1
            elif cars.v[i] >= d and cars.y[i] < cars.lanes:
                d2 = cars.distance(i, cars.y[i]+1)
                if d2 > d:
                    cars.y[i] +=1
                    cars.v[i] = d2-1
            elif cars.v[i] < d and cars.y[i] > 1:
                d3 = cars.distance(i, cars.y[i]-1)
                if d3 > cars.v[i]:
                    cars.y[i] -=1
        for i in range(cars.numCars):
            d = cars.distance(i, cars.y[i])
            if cars.v[i] >= d:
                cars.v[i] = d-1

        for i in range(cars.numCars):
            if rng.random() < self.p and cars.v[i] > 0:
                cars.v[i]-=1
        for i in range(cars.numCars):
            cars.x[i] += cars.v[i]
        cars.t +=1
        return sum(cars.v)/cars.roadLength

def draw_cars(cars, cars_drawing):

    """ Used later on to generate the animation """
    theta = []
    r     = []

    for ii in range(cars.numCars):
        position = cars.x[ii]
        # Convert to radians for plotting only (do not use radians for the simulation!)
        theta.append(position * 2 * math.pi / cars.roadLength)
        r.append(0.8+cars.y[ii]/5)

    return cars_drawing.scatter(theta, r, c=cars.c, cmap='hsv', clip_on=False)


def animate(framenr, cars, obs, propagator, road_drawing, stepsperframe):

    """ Animation function which integrates a few steps and return a drawing """

    for it in range(stepsperframe):
        propagator.propagate(cars, obs)

    return draw_cars(cars, road_drawing),


class Simulation:

    def reset(self, cars=Cars()) :
        self.cars = cars
        self.obs = Observables()

    def __init__(self, cars=Cars()) :
        self.reset(cars)

    def plot_observables(self, title="simulation"):
        plt.clf()
        plt.plot(self.obs.time, self.obs.flowrate)
        plt.xlabel('time')
        plt.ylabel('flow rate')
        #plt.savefig(title + ".pdf")
        plt.show()

    # Run without displaying any animation (fast)
    def run(self,
            propagator,
            numsteps=200,           # final time
            title="simulation",     # Name of output file and title shown at the top
            ):

        for it in range(numsteps):
            propagator.propagate(self.cars, self.obs)

        #self.plot_observables(title)

    # Run while displaying the animation of bunch of cars going in circe (slow-ish)
    def run_animate(self,
            propagator,
            numsteps=200,           # Final time
            stepsperframe=1,        # How many integration steps between visualising frames
            ):

        numframes = int(numsteps / stepsperframe)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='polar', clip_on=False)
        ax.plot(np.linspace(0, 2*np.pi, 100), np.ones(100), color='k', linestyle='-.')
        ax.plot(np.linspace(0, 2*np.pi, 100), np.ones(100)*1.2, color='k', linestyle='-.')
        ax.set_ylim([0,1.5])
        ax.axis('off')
        # Call the animator, blit=False means re-draw everything
        anim = animation.FuncAnimation(plt.gcf(), animate,  # init_func=init,
                                       fargs=[self.cars,self.obs,propagator,ax,stepsperframe],
                                       frames=numframes, interval=1000
                                       , blit=True, repeat=False)
        plt.show()

        # If you experience problems visualizing the animation and/or
        # the following figures comment out the next line 
        # plt.waitforbuttonpress(30)

        #self.plot_observables()
    

# It's good practice to encapsulate the script execution in 
# a main() function (e.g. for profiling reasons)
def main() :

    # Here you can define one or more instances of cars, with possibly different parameters, 
    # and pass them to the simulator 

    # Be sure you are passing the correct initial conditions!
    cars = Cars(numCars = 15, roadLength=50)

    # Create the simulation object for your cars instance:
    simulation = Simulation(cars)

    # simulation.run_animate(propagator=ConstantPropagator())
    simulation.run_animate(propagator=MyPropagator(vmax=5, p=0.2))
    """ simulation.plot_observables()
    plt.figure()
    plt.xlabel("position")
    plt.ylabel("time")
    colors = ["r", "b", "g", "y", "c", "m"]
    position = simulation.obs.pos
    for i in range(len(position)):
        for n in range(len(position[i])):
            plt.plot(position[i][n],simulation.obs.time[i], ".", color=colors[n])
    plt.tight_layout()
    plt.show() """

# Calling 'main()' if the script is executed.
# If the script is instead just imported, main is not called (this can be useful if you want to
# write another script importing and utilizing the functions and classes defined in this one)


main()
