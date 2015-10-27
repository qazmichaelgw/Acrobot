from config import *
from sets import Set
import numpy as np
IDX = 0
TIME = 1
POS_X = 2
class CarModel:
    def __init__(self):
        #for solver2
        self.v = {}
        self.t = {}
        self.enough_info = False
        self.car_tuple = {}
        self.speeds = {}
        self.intervals = {}

        for i in range(0, NUM_LANES):
            self.v[i] = -1
            self.t[i] = -1
            #idx time posX
            self.car_tuple[i] = (-1,-1,-1)
            self.speeds[i] = []
            self.intervals[i] = []

        #for solver3 this maybe not very good structure, may need refactoring later
        self.initial_times = {}
        self.car_ids = {}
        self.car_pos_xs = {}
        self.initial_pos_xs = {}
        self.info = {}
        self.speed_param = {}
        self.interval_param = {}
        for lane in range(0, NUM_LANES):
            self.initial_times[lane] = []
            self.car_ids[lane] = Set([])
            self.car_pos_xs[lane] = {}
            self.initial_pos_xs[lane] = -1
            self.info[lane] = [False, False]
            self.speed_param[lane] = (-1,-1)
            self.interval_param[lane] = (-1,-1)

    def printModel(self):
        print 'EnoughInfo: {0}\n speed: {1}\n time interval: {2} car_tuple: {3}\n'.format(self.enough_info, self.v, self.t, self.car_tuple)

    def getLane(self, y):
        return int(round((y-0.05)/0.1))

    def updateEnoughInfo(self):
        for lane in range(NUM_LANES):
            if self.v[lane] == -1 or self.t[lane] == -1:
                self.enough_info = False
                return
        self.enough_info = True

    def updateSolver2(self, incremental_history):
        keys = sorted(incremental_history.keys())
        for t in keys:
            current_car_info = incremental_history[t]
            if current_car_info.shape[0] > 0:
                for row in current_car_info:
                    lane = self.getLane(row[2])
                    #update t
                    if (self.t[lane] == -1 and self.car_tuple[lane][IDX] != -1 and self.car_tuple[lane][IDX] != row[0]):
                        self.t[lane] = t - self.car_tuple[lane][TIME]

                    #update car_time
                    if (self.car_tuple[lane][TIME] == -1 and self.car_tuple[lane][IDX] == -1):
                        self.car_tuple[lane] = (row[0], t, row[1])

                    #update speed
                    if (self.v[lane] == -1 and self.car_tuple[lane][TIME] != -1 and self.car_tuple[lane][POS_X] != row[1]):
                        self.v[lane] = row[1] - self.car_tuple[lane][POS_X]

        self.updateEnoughInfo()

    def checkValid(self, dict_in, lane, value):
        if len(dict_in[lane]) == 0 or dict_in[lane][-1] != value:
            return True
        return False

    def getGaussianParams(self):
        speed_param = {}
        interval_param = {}
        for lane in range(0, NUM_LANES):
            if len(self.speeds[lane]) > SAMPLES:
                speed_param[lane] = self.speed_param[lane]
            elif len(self.speeds[lane]) > 0:
                speed_param[lane] = (np.mean(self.speeds[lane]), np.sqrt(np.var(self.speeds[lane])))
                self.speed_param[lane] = speed_param[lane]
            else:
                speed_param[lane] = (-1,-1)

            if len(self.intervals[lane]) > SAMPLES:
                interval_param[lane] = self.interval_param[lane]
            elif len(self.intervals[lane]) > 0:
                interval_param[lane] = (np.mean(self.intervals[lane]), np.sqrt(np.var(self.intervals[lane])))
                self.interval_param[lane] = interval_param[lane]
            else:
                interval_param[lane] = (-1,-1)
        return speed_param, interval_param

    def updateSolver3(self, incremental_history):
        keys = sorted(incremental_history.keys())
        for t in keys:
            current_car_info = incremental_history[t]
            if current_car_info.shape[0] > 0:
                for row in current_car_info:
                    lane = self.getLane(row[2])

                    #update car_time
                    if row[0] not in self.car_ids[lane]:
                        self.initial_times[lane].append(t)
                        self.initial_pos_xs[lane] = row[1]
                        self.car_ids[lane].add(row[0])
                        #reset car_pos_x
                        self.car_pos_xs[lane] = {}
                        for lane in range(0, NUM_LANES):
                            for key in self.car_pos_xs[lane].keys():
                                speed_history = [self.car_pos_xs[lane][key][i+1] - self.car_pos_xs[lane][key][i] for i in range(len(self.car_pos_xs[lane][key])-1)]
                                if not self.info[lane][0]:
                                    self.speeds[lane] = self.speeds[lane] + speed_history
                            interval_history = [(self.initial_times[lane][i+1] - self.initial_times[lane][i])*GRID_T for i in range(len(self.initial_times[lane])-1)]
                            if not self.info[lane][1]:
                                self.intervals[lane] = self.intervals[lane] + interval_history

                    if row[0] not in self.car_pos_xs[lane].keys():
                        self.car_pos_xs[lane][row[0]] = [row[1]]
                    else:
                        self.car_pos_xs[lane][row[0]].append(row[1])
