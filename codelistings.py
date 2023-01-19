def timestep(self, cars, obs):
        for i in range(cars.numCars):
            if cars.v[i]<cars.vmax[i]:
                cars.v[i]+=1

        for i in range(cars.numCars):
            dist_forw,dist_back,vel_back = cars.distance(i, cars.y[i])
            if cars.y[i] == cars.numLanes and cars.v[i] >= dist_forw:
                cars.v[i] = dist_forw-1

            elif cars.v[i] >= dist_forw and cars.y[i] < cars.numLanes:
                dist_forw_out, dist_back_out,vel_back_out = cars.distance(i, cars.y[i]+1)
                if dist_forw_out > dist_forw and dist_back_out > vel_back_out:
                    cars.y[i] +=1
                    cars.v[i] = min(dist_forw_out-1, cars.v[i])
                else: cars.v[i] = min(dist_forw-1, cars.v[i])

            elif cars.v[i] < dist_forw and cars.y[i] > 1:
                dist_forw_in,dist_back_in,vel_back_in = cars.distance(i, cars.y[i]-1)
                if dist_forw_in > cars.v[i] and dist_back_in>vel_back_in:
                    cars.y[i] -=1

        for i in range(cars.numCars):
            if random.random() < self.p and cars.v[i] > 0:
                cars.v[i]-=1

        for i in range(cars.numCars):
            cars.x[i] += cars.v[i]
        cars.t +=1
        return sum(cars.v)/cars.roadLength