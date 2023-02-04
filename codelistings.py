
class Cars:
    """ Class for the state of a number of cars """

    def __init__(self, nCars, roadLength, nLanes, vmax):
        

    def check_surroundings(self, i, lane):
        """Returns a tuple with distance to closest car traveling in given lane ahead/aside of car i, distance to closest car traveling in given lane behind/aside car i and the velocity of that car (closest car behind in given lane)."""


def normalVelocities(nCars, vroad=5, scale=1):
    vmax = []
    for i in range(nCars):
        rnd_vel = int(round(np.random.normal(loc=vroad, scale=scale)))
        if rnd_vel >=1: vmax.append(rnd_vel)
        else: vmax.append(1)
    return vmax

class BasePropagator:

    def __init__(self):
        return
        
    def propagate(self, cars, obs):
        """ Perform a single integration step """
              
    def timestep(self, cars, obs):
        """ Virtual method: implemented by the child classes """
        pass
      
        
class Propagator(BasePropagator) :

    def __init__(self, p):
        BasePropagator.__init__(self)
        self.p = p

    def timestep(self, cars, obs):
            for i in range(cars.nCars):
                if cars.v[i]<cars.vmax[i]:
                    cars.v[i]+=1

            for i in range(cars.nCars):
                dist_forw,dist_back,vel_back = cars.check_surroundings(i, cars.y[i])
                if cars.y[i] == cars.numLanes and cars.v[i] >= dist_forw:
                    cars.v[i] = dist_forw-1

                elif cars.v[i] >= dist_forw and cars.y[i] < cars.numLanes:
                    dist_forw_out, dist_back_out,vel_back_out = cars.check_surroundings(i, cars.y[i]+1)
                    if dist_forw_out > dist_forw and dist_back_out > vel_back_out:
                        cars.y[i] +=1
                        cars.v[i] = min(dist_forw_out-1, cars.v[i])
                    else: cars.v[i] = min(dist_forw-1, cars.v[i])

                elif cars.v[i] < dist_forw and cars.y[i] > 1:
                    dist_forw_in,dist_back_in,vel_back_in = cars.check_surroundings(i, cars.y[i]-1)
                    if dist_forw_in > cars.v[i] and dist_back_in>vel_back_in:
                        cars.y[i] -=1

            for i in range(cars.nCars):
                if random.random() < self.p and cars.v[i] > 0:
                    cars.v[i]-=1

            for i in range(cars.nCars):
                cars.x[i] += cars.v[i]
            cars.t +=1
            return sum(cars.v)/cars.roadLength