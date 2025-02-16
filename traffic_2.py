#!/bin/python3

import math
import matplotlib.pyplot as plt
from matplotlib import animation
import numpy.random as rng
import numpy as np
import statistics
import matplotlib
import scipy.stats as st

font = {'family': "DejaVu Sans",
        'weight' : 'normal',
        'size'   : 20}
plt.rc('font', **font)


def normalVelocities(numCars, speedLimit, scale=1):
    vmax = []
    for i in range(numCars):
        rndv = int(round(np.random.normal(loc=speedLimit, scale=scale)))
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
            self.v.append(v0)       
            self.c.append(i)
            if i<self.roadLength:        
                self.y.append(1)  
            elif i<2*self.roadLength:
                if numLanes<2: raise KeyError
                self.y.append(2)
            else:
                if numLanes<3: raise KeyError 
                self.y.append(3)        


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
                max_vel[pos] = self.v[ii]
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
        self.flow_in_lane = [[],[],[],[],[],[]]

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
            v = []
            for j in range(cars.numCars):
                if cars.y[j] == i+1: 
                    n+=1
                    v.append(cars.v[j])
            obs.prop_in_lane[i].append(n/cars.numCars)
            obs.flow_in_lane[i].append(np.mean(v))
              
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
    numCars=20
    speedLimit=10
    vmax = normalVelocities(numCars, speedLimit, scale=1)
    cars = Cars(numCars = numCars, roadLength=200, numLanes=1, vmax=vmax)
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
    numLanes=3
    vmax = normalVelocities(2*roadLength, speedLimit)
    vel_in_lane = [[],[],[]]
    prop_in_lane = [[], [], []]
    for ncars in range(1,int(1.8*roadLength),10):
        density.append(ncars/roadLength)
        vel = [[], [], []]
        prop = [[], [], []]
        for j in range(10):
            cars = Cars(numCars = ncars, roadLength=roadLength, vmax=vmax[:ncars], numLanes=numLanes)
            sim = Simulation(cars)
            sim.run(propagator=Propagator(p=0.2))
            for l in range(numLanes):
                vel[l].append(statistics.mean(sim.obs.flow_in_lane[l][-100:-1]))
                prop[l].append(statistics.mean(sim.obs.prop_in_lane[l][-100:-1]))
        for l in range(numLanes):
            vel_in_lane[l].append(np.mean(vel[l]))
            prop_in_lane[l].append(np.mean(prop[l]))

    plt.figure()
    plt.grid()
    text = ["1st lane", "2nd lane", "3rd lane"]
    for l in range(numLanes):
        plt.plot(density, vel_in_lane[l], linestyle='--', marker='o',label=text[l])
    plt.xlabel("Density")
    plt.ylabel("Mean velocity")
    plt.legend()

    plt.figure()
    plt.grid()
    text = ["1st lane", "2nd lane", "3rd lane"]
    for l in range(numLanes):
        plt.plot(density, prop_in_lane[l], linestyle='--', marker='o',label=text[l])
    plt.xlabel("Density")
    plt.ylabel("Proportion of cars")
    plt.legend()
    
    plt.tight_layout()
    plt.show()



def trafficc():
    ste = [[], [],[]]
    N = []
    roadLength=50
    speedLimit=5
    for n in range(5, 120, 10):
        inlane = [[],[],[]]
        N.append(n)
        for i in range(n):
            vmax = normalVelocities(roadLength, speedLimit)  
            cars = Cars(numCars = 25, roadLength=50, vmax=vmax, numLanes=2)
            sim = Simulation(cars)
            sim.run(propagator=Propagator(p=0.2))
            inlane[0].append(statistics.mean(sim.obs.prop_in_lane[0][-100:-1]))
            inlane[1].append(statistics.mean(sim.obs.flow_in_lane[0][-100:-1]))
            inlane[2].append(statistics.mean(sim.obs.flow_in_lane[1][-100:-1]))
        ste[0].append(np.std(inlane[0])/(n-1)**0.5)
        ste[1].append(np.std(inlane[1])/(n-1)**0.5)
        ste[2].append(np.std(inlane[2])/(n-1)**0.5)

    N = np.array(N)
    st = np.array(ste[0])
    st2 = np.array(ste[1])
    st3 = np.array(ste[2])
    plt.figure()
    plt.grid(True, which="Both")
    plt.loglog(N,st,linestyle='--', marker='o', label="Proportion")
    plt.loglog(N,10**(-1.5)*N**(-0.5), label=r"$kN^{-0.5}$")
    plt.tight_layout()
    plt.xlabel("N")
    plt.legend()
    plt.ylabel("SEM, proportion inner lane")


    plt.figure()
    plt.grid(True, which="Both")
    plt.loglog(N,st2,linestyle='--', marker='o', label="Inner lane")
    plt.loglog(N,st3,linestyle='--', marker='o', label="Outer lane")
    plt.loglog(N, 0.1*N**(-0.5), label=r"$kN^{-0.5}$")
    plt.legend()
    plt.xlabel("N")
    plt.ylabel("SEM, mean velocity")
    plt.tight_layout()
    plt.show()   


trafficc()
