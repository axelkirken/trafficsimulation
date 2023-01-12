#!/bin/python3

import math
import matplotlib.pyplot as plt
from matplotlib import animation
import numpy.random as rng
import numpy as np
import statistics
import matplotlib
import scipy.stats as st

font = {'weight' : 'normal',
        'size'   : 16}
plt.rc('font', **font)


def normalVelocities(numCars, speedLimit):
    vmax = []
    for i in range(numCars):
        rndv = int(round(np.random.normal(loc=speedLimit, scale=1)))
        if rndv >=1: vmax.append(rndv)
        else: vmax.append(1)
    return vmax


class Cars:
    """ Class for the state of a number of cars """

    def __init__(self, numCars=5, roadLength=50, v0=1, numLanes=2, vmax=5*np.ones(5)):
        self.numCars = numCars
        self.roadLength = roadLength
        self.numLanes= numLanes
        self.t  = 0
        self.x  = []
        self.y = []
        self.v  = []
        self.vmax = vmax
        self.c  = []
        for i in range(numCars):
            self.x.append(i)        
            self.y.append(1)
            self.v.append(v0)       
            self.c.append(i)        


    def distance(self, i, lane):
        """Returns a tuple with distance to closest car traveling in given lane ahead/aside of car i, distance to closest car traveling in given lane behind/aside car i and the velocity of that car (closest car behind in given lane)."""
        x = self.x
        y = self.y
        rl = self.roadLength
        car_pos = x[i]%rl
        cars_in_lane = []
        max_vel = {}
        for ii in range(self.numCars):
            if y[ii] == lane:
                pos = x[ii]%rl
                cars_in_lane.append(pos)
                max_vel[pos] = self.vmax[ii]
        cars_in_lane = np.array(cars_in_lane)

        if cars_in_lane.size == 0:
            return rl, rl, 0

        cars_in_lane = np.sort(cars_in_lane)
        j = np.searchsorted(cars_in_lane, car_pos)
        """ if st.mode(cars_in_lane, keepdims=False)[1] > 1:
            print(cars_in_lane) """

        if lane == y[i]:
            if cars_in_lane.size == 1:
                return rl, rl, max_vel[cars_in_lane[0]]
            if car_pos == np.max(cars_in_lane):
                return rl-(car_pos-cars_in_lane[0]), car_pos-cars_in_lane[j-1], None
            if car_pos == np.min(cars_in_lane): 
                return cars_in_lane[j+1]-car_pos, rl-(cars_in_lane[-1]-car_pos), None
            return cars_in_lane[j+1]-car_pos, car_pos-cars_in_lane[j-1], None
        else: 
            if car_pos >= np.max(cars_in_lane):
                if car_pos == np.max(cars_in_lane): return 0,0,max_vel[car_pos]
                return rl-(car_pos-cars_in_lane[0]), car_pos-cars_in_lane[j-1], max_vel[cars_in_lane[j-1]]
            elif car_pos <= np.min(cars_in_lane): 
                if car_pos == np.min(cars_in_lane): return 0,0,max_vel[car_pos]
                return cars_in_lane[j]-car_pos, rl-(cars_in_lane[-1]-car_pos), max_vel[cars_in_lane[-1]]
            else:
                return cars_in_lane[j]-car_pos, car_pos-cars_in_lane[j-1], max_vel[cars_in_lane[j-1]]
            
            
class Observables:
    """ Class for storing observables """
    def __init__(self):
        self.time = []          # list to store time
        self.flowrate = []      # list to store the flow rate
        self.prop_in_lane = [[],[],[],[],[],[]]

class BasePropagator:

    def __init__(self):
        return
        
    def propagate(self, cars, obs):
        """ Perform a single integration step """
        
        fr = self.timestep(cars, obs)
        obs.time.append(cars.t)
        obs.flowrate.append(fr)
        for i in range(cars.numLanes):
            n=0
            for j in range(cars.numCars):
                if cars.y[j] == i+1: n+=1
            obs.prop_in_lane[i].append(n/cars.numCars)
              
    def timestep(self, cars, obs):
        """ Virtual method: implemented by the child classes """
        pass
      
        
class Propagator(BasePropagator) :

    def __init__(self, p):
        BasePropagator.__init__(self)
        self.p = p

    def timestep(self, cars, obs):
        
        for i in range(cars.numCars):
            if cars.v[i]<cars.vmax[i]:
                cars.v[i]+=1

        for i in range(cars.numCars):
            df,db,vb = cars.distance(i, cars.y[i])
            if cars.y[i] == cars.numLanes and cars.v[i]>=df:
                cars.v[i] = df-1

            elif cars.v[i] >= df and cars.y[i] < cars.numLanes:
                d2f, d2b,v2b = cars.distance(i, cars.y[i]+1)
                if d2f > df and d2b > v2b:
                    cars.y[i] +=1
                    cars.v[i] = min(d2f-1, cars.v[i])
                else: cars.v[i] = min(df-1, cars.v[i])

            elif cars.v[i] < df and cars.y[i] > 1:
                d3f,d3b,v3b = cars.distance(i, cars.y[i]-1)
                if d3f > cars.v[i] and d3b>v3b:
                    cars.y[i] -=1
            
        for i in range(cars.numCars):
            assert cars.v[i]<cars.distance(i, cars.y[i])[0]

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
        theta.append(2*math.pi - position * 2 * math.pi / cars.roadLength)
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
        plt.show()


    def run(self,propagator,numsteps=200):
        for it in range(numsteps):
            propagator.propagate(self.cars, self.obs)


    def run_animate(self,propagator,numsteps=200,stepsperframe=1):
        numframes = int(numsteps / stepsperframe)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='polar', clip_on=False)
        for i in range(1,self.cars.numLanes+1):
            ax.plot(np.linspace(0, 2*np.pi, 100), (0.8+i/5)*np.ones(100), color='k', linestyle='-.')
        ax.set_ylim([0,1+i/5])
        ax.axis('off')
        anim = animation.FuncAnimation(plt.gcf(), animate, 
                                       fargs=[self.cars,self.obs,propagator,ax,stepsperframe],
                                       frames=numframes, interval=200
                                       , blit=True, repeat=False)
        plt.show()


def main() :
    numCars=40
    speedLimit=5
    vmax = normalVelocities(numCars, speedLimit)
    cars = Cars(numCars = numCars, roadLength=200, numLanes=3, vmax=vmax)
    plt.figure()
    bins = np.arange(1, max(vmax) + 1.5) - 0.5
    plt.hist(vmax, bins=bins, rwidth=0.8)
    plt.show()
    sim = Simulation(cars)
    sim.run_animate(propagator=Propagator(p=0.2))

    
def traffica():
    density = []
    roadLength=100
    speedLimit=5
    vmax = normalVelocities(roadLength, speedLimit)
    in_lane = [[], []]
    for ncars in range(1,roadLength,5):
        density.append(ncars/roadLength)
        cars = Cars(numCars = ncars, roadLength=roadLength, vmax=vmax[:ncars], numLanes=2)
        sim = Simulation(cars)
        sim.run(propagator=Propagator(p=0.2))
        in_lane[0].append(statistics.mean(sim.obs.prop_in_lane[0][-100:-1]))
        in_lane[1].append(statistics.mean(sim.obs.prop_in_lane[1][-100:-1]))
    plt.figure()
    plt.plot(density, in_lane[0], label="Inner lane")
    plt.plot(density, in_lane[1], label="Outer lane")
    plt.legend()
    plt.xlabel("density")
    plt.ylabel("Proportion in lane")
    plt.tight_layout()
    plt.show()


def trafficb():
    density = []
    roadLength=100
    speedLimit=5
    vmax = normalVelocities(2*roadLength, speedLimit)
    flow = []
    flow2 = []
    flow3 = []
    for ncars in range(1,2*roadLength,10):
        density.append(ncars/roadLength)
        cars = Cars(numCars = ncars, roadLength=roadLength, numLanes=1, vmax=vmax[:ncars])
        cars2 = Cars(numCars = ncars, roadLength=roadLength, numLanes=2, vmax=vmax[:ncars])
        cars3 = Cars(numCars = ncars, roadLength=roadLength, numLanes=3, vmax=vmax[:ncars])
        sim = Simulation(cars)
        sim2 = Simulation(cars2)
        sim3 = Simulation(cars3)
        sim.run(propagator=Propagator(p=0.2))
        sim2.run(propagator=Propagator(p=0.2))
        sim3.run(propagator=Propagator(p=0.2))
        flow.append(statistics.mean(sim.obs.flowrate[-100:-1]))
        flow2.append(statistics.mean(sim2.obs.flowrate[-100:-1]))
        flow3.append(statistics.mean(sim3.obs.flowrate[-100:-1]))
    plt.figure()
    plt.plot(density, flow, label="One lane")
    plt.plot(density, flow2, label="Two lanes")
    plt.plot(density, flow3, label="Three lanes")
    plt.legend()
    plt.xlabel("density")
    plt.ylabel("flow rate")
    plt.tight_layout()
    plt.show()



def trafficc():
    ste = []
    N = []
    roadLength=50
    speedLimit=5
    vmax = normalVelocities(roadLength, speedLimit)
    for n in range(5, 120, 10):
        flow = np.array([])
        N.append(n)
        for i in range(n):
            cars = Cars(numCars = 10, roadLength=50, vmax=vmax, numLanes=2)
            sim = Simulation(cars)
            sim.run(propagator=Propagator(p=0.2))
            flow = np.append(flow, [statistics.mean(sim.obs.flowrate[-100:-1])])
        ste.append(np.std(flow)/(n-1)**0.5)
    plt.figure()
    plt.plot(N,ste)
    plt.xlabel("Number of simulations")
    plt.ylabel("Standard error of flow rate")
    plt.tight_layout()
    plt.show()   


def trafficd():
    roadLength=50
    speedLimit=5
    vmax = normalVelocities(roadLength, speedLimit)
    plt.figure()
    for i in range(1,6):
        cars = Cars(numCars = 10, roadLength=50, vmax=vmax, numLanes=i)
        sim = Simulation(cars)
        sim.run(propagator=Propagator(p=0.2))
        plt.plot(sim.obs.time, sim.obs.flowrate, label=i)
    plt.legend()
    plt.show()


traffica()
