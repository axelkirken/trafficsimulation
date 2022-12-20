import numpy as np

arr = np.array([1, 2, 3, 4, 5, 4, 4])

x = np.where(arr == 4)

print(arr[x])


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
                else: cars.v[i] = d-1
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



         if self.x[i] >= np.max(cars_in_lane):    
            return self.roadLength - (self.x[i] - np.min(cars_in_lane))
        else:
            dist = cars_in_lane - self.x[i]
            print(dist)
            if lane == self.y[i]:
                return np.min(dist[np.where(dist>0)]), np.abs(np.max(dist[np.where(dist<0)]))
            else: 
                return np.min(dist[np.where(dist>=0)]), np.abs(np.max(dist[np.where(dist<=0)])) 