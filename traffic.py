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

class Cars:
    """ Class for the state of a number of cars """

    def __init__(self, numCars=5, roadLength=50, v0=1, lanes=2, roadmaxv=5):
        self.numCars    = numCars
        self.roadLength = roadLength
        self.lanes= lanes
        self.roadmaxv=5
        self.t  = 0
        self.x  = []
        self.y = []
        self.v  = []
        self.vmax = []
        self.c  = []
        for i in range(numCars):
            self.x.append(i)        
            self.y.append(1)
            self.v.append(v0)       
            rndv = int(round(np.random.normal(loc=roadmaxv, scale=1)))
            if rndv >=1: self.vmax.append(rndv)
            else: self.vmax.append(1)
            self.c.append(i)        


    def distance(self, i, lane):
        rl = self.roadLength
        car_pos = self.x[i]%rl
        cars_in_lane = []
        vel = []
        for ii in range(self.numCars):
            if self.y[ii] == lane:
                cars_in_lane.append(self.x[ii]%rl)
                vel.append(self.vmax[ii])
        cars_in_lane = np.array(cars_in_lane)

        if cars_in_lane.size == 0:
            return self.roadLength, self.roadLength, 0, 0

        cars_in_lane = np.sort(cars_in_lane)
        j = np.searchsorted(cars_in_lane, car_pos)
        if st.mode(cars_in_lane, keepdims=False)[1] > 1:
            print(cars_in_lane)

        if lane == self.y[i]:
            if cars_in_lane.size == 1:
                return self.roadLength, self.roadLength, vel[0], vel[0]
            if car_pos == np.max(cars_in_lane):
                return self.roadLength-(cars_in_lane[-1]-cars_in_lane[0]),car_pos-cars_in_lane[j-1], vel[0], vel[j-1]
            if car_pos == np.min(cars_in_lane): 
                return cars_in_lane[j+1]-car_pos, self.roadLength-(cars_in_lane[-1]-cars_in_lane[0]), vel[1], vel[-1]
            return cars_in_lane[j+1]-car_pos, car_pos-cars_in_lane[j-1], vel[j+1], vel[j-1]
        else: 
            if car_pos >= np.max(cars_in_lane):
                if car_pos == np.max(cars_in_lane): return 0,0,0,0
                return self.roadLength-(car_pos-cars_in_lane[0]), car_pos-cars_in_lane[j-1], vel[0], vel[j-1]
            elif car_pos <= np.min(cars_in_lane): 
                if car_pos == np.min(cars_in_lane): return 0,0,0,0
                return cars_in_lane[j]-car_pos, self.roadLength-(cars_in_lane[-1]-car_pos), vel[j], vel[-1]
            else:
                return cars_in_lane[j]-car_pos, car_pos-cars_in_lane[j-1], vel[j], vel[j-1]
            
            
class Observables:

    """ Class for storing observables """

    def __init__(self):
        self.time = []          # list to store time
        self.flowrate = []      # list to store the flow rate
        self.proplane = [[], [], []]

class BasePropagator:

    def __init__(self):
        return
        
    def propagate(self, cars, obs):

        """ Perform a single integration step """
        
        fr = self.timestep(cars, obs)

        obs.time.append(cars.t)
        obs.flowrate.append(fr)
        for i in range(cars.lanes):
            n=0
            for j in range(cars.numCars):
                if cars.y[j] == i+1: n+=1
            obs.proplane[i].append(n/cars.numCars)
              
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
            df, db,vf,vb = cars.distance(i, cars.y[i])
            if cars.y[i] == cars.lanes and cars.v[i]>=df:
                cars.v[i] = df-1

            elif cars.v[i] >= df and cars.y[i] < cars.lanes:
                d2f, d2b,vf,v2b = cars.distance(i, cars.y[i]+1)
                if d2f > df and d2b > v2b:
                    cars.y[i] +=1
                    cars.v[i] = min(d2f-1, cars.vmax[i])
                else: cars.v[i] = min(df-1, cars.vmax[i])

            elif cars.v[i] < df and cars.y[i] > 1:
                d3f, d3b,vf,v3b = cars.distance(i, cars.y[i]-1)
                if d3f > cars.v[i] and d3b>v3b:
                    cars.y[i] -=1
        
        for i in range(cars.numCars):
            d,a,b,c = cars.distance(i, cars.y[i])
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
        plt.show()


    def run(self,propagator,numsteps=200):
        for it in range(numsteps):
            propagator.propagate(self.cars, self.obs)


    def run_animate(self,propagator,numsteps=200,stepsperframe=1):
        numframes = int(numsteps / stepsperframe)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='polar', clip_on=False)
        for i in range(1,self.cars.lanes+1):
            ax.plot(np.linspace(0, 2*np.pi, 100), (0.8+i/5)*np.ones(100), color='k', linestyle='-.')
        ax.set_ylim([0,1.5])
        ax.axis('off')
        anim = animation.FuncAnimation(plt.gcf(), animate, 
                                       fargs=[self.cars,self.obs,propagator,ax,stepsperframe],
                                       frames=numframes, interval=200
                                       , blit=True, repeat=False)
        plt.show()


def main() :

    cars = Cars(numCars = 15, roadLength=200, lanes=3, roadmaxv=5)
    plt.figure()
    plt.hist(cars.vmax, density=True, bins=10)
    plt.show()

    sim = Simulation(cars)
    sim.run_animate(propagator=Propagator(p=0.2))

    
  
def traffica():
    density = []
    roadLength=100
    inner = []
    for ncars in range(1,roadLength):
        density.append(ncars/roadLength)
        cars = Cars(numCars = ncars, roadLength=roadLength, roadmaxv=5)
        sim = Simulation(cars)
        sim.run(propagator=Propagator(p=0.2))
        inner.append(statistics.mean(sim.obs.proplane[0][-100:-1]))
    plt.figure()
    plt.plot(density, inner)
    plt.xlabel("density")
    plt.ylabel("Proportion in inner lane")
    plt.tight_layout()
    plt.show()



main()
