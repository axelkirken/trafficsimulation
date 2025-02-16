#!/bin/python3

import math
import matplotlib.pyplot as plt
from matplotlib import animation
import numpy.random as rng
import numpy as np
import statistics
import matplotlib
import scipy.stats as st
import scipy as sc
from scipy.stats import t
tinv = lambda p, df: abs(t.ppf(p/2, df))

font = {'family': "DejaVu Sans",
        'weight' : 'normal',
        'size'   : 22}
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
        #self.flow_in_lane = [[],[],[],[],[],[]]

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
            #v = []
            for j in range(cars.numCars):
                if cars.y[j] == i+1: 
                    n+=1
                    #v.append(cars.v[j])
            obs.prop_in_lane[i].append(n/cars.numCars)
            #obs.flow_in_lane[i].append(np.mean(v))
              
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


def main():
    numCars=100
    speedLimit=5
    vmax = normalVelocities(numCars, speedLimit, scale=1)
    cars = Cars(numCars = numCars, roadLength=200, numLanes=3, vmax=vmax)
    plt.figure()
    bins = np.arange(1, max(vmax) + 1.5) - 0.5
    plt.hist(vmax, bins=bins, rwidth=0.8)
    plt.show()
    sim = Simulation(cars)
    sim.run_animate(propagator=Propagator(p=0.2))


def trafficb():
    density = np.array([])
    roadLength=100
    speedLimit=5
    flow = [[],[],[]]
    for ncars in range(1,int(0.3*roadLength),4):
        density = np.append(density, ncars/roadLength)
        for i in range(3):
            if ncars <= (i+1)*roadLength:
                flowtemp = []
                for j in range(10):
                    vmax = normalVelocities(3*roadLength, speedLimit)
                    cars = Cars(numCars = ncars, roadLength=roadLength, numLanes=i+1, vmax=vmax[:ncars])
                    sim = Simulation(cars)
                    sim.run(propagator=Propagator(p=0.2))
                    flowtemp.append(statistics.mean(sim.obs.flowrate[-100:-1]))
                flow[i].append(np.mean(flowtemp))
    
    for ncars in range(int(0.3*roadLength), 3*roadLength, 10):
        density = np.append(density, ncars/roadLength)
        for i in range(3):
            if ncars <= (i+1)*roadLength:
                flowtemp = []
                for j in range(10):
                    vmax = normalVelocities(3*roadLength, speedLimit)
                    cars = Cars(numCars = ncars, roadLength=roadLength, numLanes=i+1, vmax=vmax[:ncars])
                    sim = Simulation(cars)
                    sim.run(propagator=Propagator(p=0.2))
                    flowtemp.append(statistics.mean(sim.obs.flowrate[-100:-1]))
                flow[i].append(np.mean(flowtemp))


    SStot = [0,0,0]
    plt.figure()
    plt.grid()
    colors = ["g", "c", "m"]
    s=""
    for i in range(3):
        mean = np.mean(flow[i])
        for y in flow[i]:
            SStot[i]+= (y-mean)**2
        plt.plot(density[:len(flow[i])], flow[i], colors[i]+".")
        
        turnpoint = flow[i].index(max(flow[i]))
        p, res, _,_,_ = np.polyfit(density[:turnpoint], flow[i][:turnpoint], 1, full=True)
        p2, res2, _,_,_ = np.polyfit(density[turnpoint:len(flow[i])], flow[i][turnpoint:],1, full=True)
        plt.plot(density[:turnpoint+1], density[:turnpoint+1]*p[0]+p[1], colors[i]+"--",  label=str(i+1)+" lane"+s)
        s="s"
        plt.plot(density[turnpoint-1:len(flow[i])], density[turnpoint-1:len(flow[i])]*p2[0]+p2[1], colors[i]+"--")
        if res.size==0: res=[0]
        restot = res[0] + res2[0]
        print(1-restot/SStot[i])
        
        result = st.linregress(density[:turnpoint], flow[i][:turnpoint])
        result2 = st.linregress(density[turnpoint:len(flow[i])], flow[i][turnpoint:])
        ts1 = tinv(0.05, len(density[:turnpoint])-2)
        ts2 = tinv(0.05, len(density[turnpoint:len(flow[i])])-2)
        print(f"slope (i): {result.slope:.6f} +/- {ts1*result.stderr:.6f}")
        print(f"intercept (i): {result.intercept:.6f}"f" +/- {ts1*result.intercept_stderr:.6f}")
        print(f"slope (d): {result2.slope:.6f} +/- {ts1*result2.stderr:.6f}")
        print(f"intercept (d): {result2.intercept:.6f}"f" +/- {ts1*result2.intercept_stderr:.6f}")

    plt.tight_layout()
    plt.legend()
    plt.xlabel(r'Density ($ \rho $)')
    plt.ylabel("Flow rate (q)")
    plt.tight_layout()
    plt.show()


def f(x, a,b):
    return a*x**2+b*x

def trafficb2():
    density = np.array([])
    roadLength=50
    speedLimit=5
    vmax = normalVelocities(3*roadLength, speedLimit)
    flow = [[],[],[]]
    for ncars in range(1,3*roadLength,5):
        density = np.append(density, ncars/roadLength)
        for i in range(3):
            if ncars <= (i+1)*roadLength:
                flowtemp = []
                for j in range(10):
                    cars = Cars(numCars = ncars, roadLength=roadLength, numLanes=i+1, vmax=vmax[:ncars])
                    sim = Simulation(cars)
                    sim.run(propagator=Propagator(p=0.2))
                    flowtemp.append(statistics.mean(sim.obs.flowrate[-100:-1]))
                flow[i].append(np.mean(flowtemp))
    
    SStot = [0,0,0]
    plt.figure()
    colors = ["g", "c", "m"]
    s=""
    for i in range(3):
        mean = np.mean(flow[i])
        for y in flow[i]:
            SStot[i]+= (y-mean)**2
        x = density[:len(flow[i])]
        plt.plot(x, flow[i], colors[i]+".")
        
        p, pcov = sc.optimize.curve_fit(f, density[:len(flow[i])], flow[i])
        print(p)
        plt.plot(x, f(x, p[0],p[1]), colors[i]+"--", label=str(i+1)+" lane"+s)
        s="s"
        res = flow[i]-f(density[:len(flow[i])], p[0],p[1])
        ss_res = np.sum(res**2)
        print(1-ss_res/SStot[i])
    plt.tight_layout()
    plt.legend()
    plt.xlabel(r"Density ($\rho$)")
    plt.ylabel("Flow rate (q)")
    plt.tight_layout()
    plt.show()


def trafficc():
    ste = [[], [], []]
    N = []
    roadLength=100
    speedLimit=5
    
    for n in range(5, 120, 5):
        flow = [[],[],[]]
        N.append(n)
        for i in range(n):
            for j in range(1,4):
                vmax = normalVelocities(roadLength, speedLimit)  
                cars = Cars(numCars = 50, roadLength=roadLength, vmax=vmax, numLanes=j)
                sim = Simulation(cars)
                sim.run(propagator=Propagator(p=0.2))
                flow[j-1].append(statistics.mean(sim.obs.flowrate[-100:-1]))
        for j in range(1,4):
            ste[j-1].append(np.std(flow[j-1])/(n-1)**0.5)
    plt.figure()
    plt.grid(True, which="Both")
    s=""
    for j in range(1,4):
        N = np.array(N)
        st = np.array(ste[j-1])
        plt.loglog(N,st,linestyle='--', marker='o', label=str(j)+" lane"+s)
        s="s"
    plt.loglog(N, N**(-0.5)*10**(-1.5), linestyle='--', marker='o', label=r"$kN^{-0.5}$")
    plt.legend()
    plt.xlabel("N")
    plt.ylabel(r"$s_{\bar{q}}$")
    plt.tight_layout()
    plt.show()   


def trafficd():
    roadLength=100
    speedLimit=5
    vmax = normalVelocities(roadLength, speedLimit)
    plt.figure()
    plt.tight_layout()
    plt.grid()
    s=""
    for i in range(1,4):
        cars = Cars(numCars = 50, roadLength=roadLength, vmax=vmax, numLanes=i)
        sim = Simulation(cars)
        sim.run(propagator=Propagator(p=0.2))
        plt.plot(sim.obs.time, sim.obs.flowrate, label=str(i)+" lane"+s)
        s="s"
        print(np.mean(sim.obs.flowrate[-100:-1]), max(sim.obs.flowrate[-100:-1])-min(sim.obs.flowrate[-100:-1]))
    plt.xlabel("Time")
    plt.ylabel("Flow rate")
    plt.legend()
    plt.show()


def traffice():
    speedLimit=10
    roadLength=100
    spread = []
    flowtot = [[], [], []]
    for scale in range(0,50,5):
        spread.append(scale*0.1)
        for iter in range(20):
            flow = [[], [], []]
            for i in range(1,4):
                vmax = normalVelocities(roadLength, speedLimit=speedLimit, scale=scale*0.1)
                cars = Cars(numCars = 10, roadLength=roadLength, vmax=vmax, numLanes=i)
                sim = Simulation(cars)
                sim.run(propagator=Propagator(p=0.2))
                flow[i-1].append(statistics.mean(sim.obs.flowrate[-100:-1]))
        for j in range(3):
            flowtot[j].append(np.mean(flow[j]))
    plt.figure()
    plt.grid()
    plt.tight_layout()
    text=" lane"
    for i in range(1,4):
        plt.plot(spread, flowtot[i-1],linestyle='--', marker='o', label=str(i) +text)
        text = " lanes"
    plt.xlabel("Max velocity standard deviation")
    plt.ylabel("Flow rate")
    plt.legend()
    plt.show()


def trafficf():
    speedLimit=5
    roadLengths = [10,30,50,100,150,200]
    plt.figure()
    plt.tight_layout()
    for roadLength in roadLengths:
        density = []
        flow = []
        for ncars in range(1,int(1*roadLength),int(roadLength/10)):
            flowtemp = []
            density.append(ncars/roadLength)
            for j in range(10):
                vmax = normalVelocities(ncars, speedLimit=speedLimit)
                cars = Cars(numCars = ncars, roadLength=roadLength, vmax=vmax, numLanes=2)
                sim = Simulation(cars)
                sim.run(propagator=Propagator(p=0.2))
                flowtemp.append(statistics.mean(sim.obs.flowrate[-100:-1]))
            flow.append(np.mean(flowtemp))
        plt.plot(density, flow,"--", label="L="+str(roadLength))
    plt.xlabel(r"Density ($\rho$)")
    plt.ylabel("Flow rate (q)")
    plt.legend()
    plt.show()

main()