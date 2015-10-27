import sys
from time import time
import pandas as pd
import re
import numpy as np
from config import *
import matplotlib.pyplot as plt
import matplotlib
from priority_queue import PriorityQueue
from car_model import CarModel
def getScore(cross, death):
    print "cross: {0} death: {1} score: {2}".format(cross, death, 10*cross-death)

def loadData(file_path):
    #parse data
    test = pd.read_csv(file_path, delimiter="\n",header = None).as_matrix().flatten()
    data = np.split(test, np.where(test[:] == '#')[0][0:])
    res = [data[0]] + [data[i][1:] for i in range(1,len(data)-1)]
    key = [int(round(float(res[i][0])/GRID_T)) for i in range(len(res))]

    def parseInfo(arr):
        list = arr.tolist()
        return np.array([np.array(list[i].split(' ')[:3]).astype(float) for i in range(len(list))])

    valueTmp = map(lambda x: x[1:], res)
    value = [parseInfo(res[i][1:]) for i in range(len(res))]
    car_info = dict(zip(key, value))
    return car_info

def checkBlock(car_x):
    res = False
    if (car_x >= BLOCK_MIN_T and car_x <= BLOCK_MAX_T):
        res = True
    return res

def generateConfigurationSpace(car_info):
    keys = sorted(car_info.keys())
    block = {}
    for key in keys:
        def myfunc(row):
            return checkBlock(row[1])
        block[key] = np.array([myfunc(row) for row in car_info[key]])
    return block

def transformGridMap(car_info, block): #discretize to grid of GRID_TxGRID_Y
    keys = sorted(car_info.keys())
    res = np.zeros((NUM_GRIDS_Y,len(keys)))
    #add blocks
    for key in keys:
        if car_info[key].shape[0] > 0:
            block_center = car_info[key][block[key]]
            #consider car and chicken height
            block_center_grid = block_center[:,2]/GRID_Y
            half_height_grids = int(round((CHICKEN_H/2+CAR_H/2)/GRID_Y))
            for grid_y in block_center_grid:
                for idy in range(int(round(grid_y-half_height_grids)), int(round(grid_y+half_height_grids))):
                    res[NUM_GRIDS_BELOW_ZERO+idy][key] = 1
    return res

def aStar(grid_map, source):
    grid_max = np.amax(grid_map)
    if grid_max == 0:
        grid_max = 1
    def heuristic(current):
        (t,y) = current
        return NUM_GRIDS_Y-y

    def neighbors(current):
        res = []
        (t,y) = current
        if t+1 < grid_map.shape[1]:
            num_offset = int(round(CHICKEN_MAX_SPEED/GRID_Y))
            for yy in range(-num_offset, num_offset+1):
                neighbor = (t+1, y+yy)
                if neighbor[1] < NUM_GRIDS_Y and neighbor[1] >= 0:
                    res.append(neighbor)
        return res

    def checkGoal(current):
        (t, y) = current
        if (y >= NUM_GRIDS_Y - EXTRA_GRIDS):
            return True
        return False

    frontier = PriorityQueue()
    frontier.put(source, 0)
    came_from = {}
    cost_so_far = {}
    came_from[source] = None
    cost_so_far[source] = 0

    while not frontier.empty():
        current = frontier.pop()

        if checkGoal(current):
            break

        for next in neighbors(current):
            new_cost = cost_so_far[current] + (np.exp(COEFF*grid_map[next[1]][next[0]]/grid_max)-1)/EXP_COEFF*NUM_GRIDS_Y
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(next)
                frontier.put(next, priority)
                came_from[next] = current

    goal=sorted(came_from.keys())[-1]
    return came_from, goal

def plotGridMap(grid_map,start = 0, end = 500):
    tmp_grid = np.copy(grid_map[:, start:end])
    grid_max = np.amax(tmp_grid)
    if grid_max == 0:
        grid_max = 1
    tmp_grid = tmp_grid/grid_max
    print tmp_grid
    samples = tmp_grid*255
    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111)
    ax.set_title('configurationSpace')
    plt.imshow(samples)
    ax.set_aspect('equal')
    plt.show()

def generateAnswer(came_from, goal, file_name):
    f = open(file_name, 'a')
    path = []
    while goal != None:
        path.append(goal)
        goal = came_from[goal]
    for current in range(len(path)-1, -1, -1):
        (t,y) = path[current]
        s = '{0}\n{1} {2}\n#\n'.format(t*GRID_T,CHICKEN_X,(y-NUM_GRIDS_BELOW_ZERO)*GRID_Y)
        f.write(s)
    f.close()

def clearFile(file_name):
    f = open(file_name, 'w')
    f.write("")
    f.close()

def solver1():
    cross = 0
    death = 0
    car_info = loadData(sys.argv[1])
    configuration_space = generateConfigurationSpace(car_info)
    grid_map = transformGridMap(car_info, configuration_space)
    #print grid_map
    #plot(configuration_space, car_info)
    source = (0, 0)
    max_time_steps = sorted(car_info.keys())[-1]
    output_file = sys.argv[3]
    clearFile(output_file)
    while source[0] < max_time_steps:
        print 'Source: '+repr(source)
        came_from, goal = aStar(grid_map, source)
        generateAnswer(came_from, goal, output_file)
        if (goal[1] >= NUM_GRIDS_Y - EXTRA_GRIDS):
            cross += 1
        source = (goal[0]+1,0)
    # plot configuration space step by step
    #for time_step in range(0, max_time_steps):
    #    if time_step%1000 == 0:
    #        print time_step
    #        plotGridMap(grid_map, time_step, time_step + HORIZON)

    getScore(cross, death)
    print 'write to {0} file'.format(output_file)

#functions below used for solver2
def initialEmptyGridMap(car_info):
    keys = sorted(car_info.keys())
    res = np.zeros((NUM_GRIDS_Y,len(keys)+HORIZON))
    return res

def addBlocks(grid_map, car_info, barrier=None, withCostMap = False, current_time=None):
    block = generateConfigurationSpace(car_info)
    keys = car_info.keys()
    #add new history blocks
    for key in keys:
        if car_info[key].shape[0] > 0:
            block_center = car_info[key][block[key]]
            #consider car and chicken height
            block_center_grid = block_center[:,2]/GRID_Y
            for grid_y in block_center_grid:
                for idy in range(int(round(grid_y-HALF_HEIGHT_GRIDS)), int(round(grid_y+HALF_HEIGHT_GRIDS))):
                    if withCostMap:
                        if barrier[NUM_GRIDS_BELOW_ZERO+idy][key] == 2:
                            grid_map[NUM_GRIDS_BELOW_ZERO+idy][key] += 0.5*(key-current_time)
                        else:
                            grid_map[NUM_GRIDS_BELOW_ZERO+idy][key] += 1*(key-current_time)
                    else:
                        grid_map[NUM_GRIDS_BELOW_ZERO+idy][key] = 1


def updateGridMap(grid_map, current_time_step, incremental_car_history, car_model, enough_info):
    #add prediction blocks
    fake_car_id = 0
    POS_X = 2
    TIME = 1
    horizon = HORIZON
    if enough_info:
        horizon = grid_map.shape[1] - current_time_step
    car_prediction = {}
    for lane in range(NUM_LANES):
        speed = car_model.v[lane]
        if speed != -1:
            #add new car
            car_list = []
            if car_model.t[lane] != -1:
                for t in range(car_model.car_tuple[lane][TIME], car_model.car_tuple[lane][TIME] + current_time_step+horizon, car_model.t[lane]):
                    car_list.append(t)
            else:
                car_list = [car_model.car_tuple[lane][TIME]]
            for car_t in car_list:
                for t in range(current_time_step, current_time_step+horizon):
                    pos_x = car_model.car_tuple[lane][POS_X]+speed*(t-car_t)
                    #prune those can not be blocked
                    if pos_x >= BLOCK_MIN_T and pos_x <= BLOCK_MAX_T:
                        tmpArr = np.array([[fake_car_id, pos_x, lane*0.1+0.05]])
                        if t not in car_prediction.keys():
                            car_prediction[t] = tmpArr
                        else:
                            car_prediction[t] = np.concatenate((car_prediction[t],tmpArr))
    addBlocks(grid_map, car_prediction)

def updateGridMapSolver3(grid_map, current_time_step, incremental_car_history, car_model, speed_param, interval_param, barrier):
    #add prediction blocks
    fake_car_id = 0
    horizon = HORIZON
    car_prediction = {}
    for lane in range(NUM_LANES):
        if speed_param[lane][0] != -1:
            #add new car
            car_list = []
            if interval_param[lane][0] != -1:
                t = car_model.initial_times[lane][-1]
                while t < current_time_step + horizon:
                    t += int(round((interval_param[lane][0] + np.random.randn()*interval_param[lane][1])/GRID_T))
                    car_list.append(t)
            car_list = car_model.initial_times[lane] + car_list
            for car_t in car_list:
                if car_t < current_time_step - BLOCK_MAX_T/CAR_MIN_SPEED:
                    continue
                pos_x = car_model.initial_pos_xs[lane]
                for steps in range(0, current_time_step-car_t-1):
                    speed = speed_param[lane][0] + speed_param[lane][1]*np.random.randn()
                    pos_x = pos_x + speed
                for t in range(current_time_step, current_time_step+horizon):
                    #do not consider the range of speed in case of death loop
                    #while abs(speed) < CAR_MIN_SPEED or abs(speed) > CAR_MAX_SPEED:
                    #speed = speed_param[lane][0] + speed_param[lane][1]*np.random.randn()
                    if t-car_t > 0:
                        speed = speed_param[lane][0] + speed_param[lane][1]*np.random.randn()
                        pos_x = pos_x + speed
                    #prune those can not be blocked
                    if pos_x >= BLOCK_MIN_T and pos_x <= BLOCK_MAX_T:
                        tmpArr = np.array([[fake_car_id, pos_x, lane*0.1+0.05]])
                        if t not in car_prediction.keys():
                            car_prediction[t] = tmpArr
                        else:
                            car_prediction[t] = np.concatenate((car_prediction[t],tmpArr))
    addBlocks(grid_map, car_prediction, barrier, True, current_time_step)

def oneStepAction(grid_map, source):
    sub_goal = (-1,-1)
    came_from, goal = aStar(grid_map, source)
    #path = []
    #tmpGoal = goal
    #while tmpGoal != None:
    #    path.append(tmpGoal)
    #    tmpGoal = came_from[tmpGoal]
    #print path
    #print reachGoal

    while goal != None:
        current = came_from[goal]
        if current == source:
            sub_goal = goal
            break
        goal = current
    return came_from, sub_goal

def solver2():
    car_info = loadData(sys.argv[1])
    grid_map = initialEmptyGridMap(car_info)
    test_grid = np.copy(grid_map)
    car_model = CarModel()
    source = (0,0)
    max_time_steps = sorted(car_info.keys())[-1]
    current_time_step = 0
    output_file = sys.argv[3]
    death = 0
    cross = 0
    clearFile(output_file)
    needUpdateGrid = True
    last_valid_came_from = {}
    f = open(output_file, 'a')
    (t, y) = source
    s = '{0}\n{1} {2}\n#\n'.format(0*GRID_T,CHICKEN_X,(0-NUM_GRIDS_BELOW_ZERO)*GRID_Y)
    f.write(s)
    f.close()
    while current_time_step < max_time_steps:
        incremental_car_history = {}
        incremental_car_history[current_time_step] = car_info[current_time_step]
        if car_model.enough_info == False:
            print 'Online Source: '+repr(source)
            car_model.updateSolver2(incremental_car_history )
            #car_model.printModel()
            updateGridMap(grid_map, current_time_step, incremental_car_history, car_model, car_model.enough_info)
            #plotGridMap(grid_map, current_time_step, current_time_step + 500)
            came_from, sub_goal = oneStepAction(grid_map, source)
            #only need one step and no overlab then just transfer in source
            if sub_goal == (-1,-1):
                goal = last_valid_came_from[sorted(last_valid_came_from.keys())[-1]]
                while goal != None:
                    current = last_valid_came_from[goal]
                    if current[0] == current_time_step+1:
                        source = current
                        break
                    goal = current
            else:
                last_valid_came_from = came_from
                source = sub_goal
            f = open(output_file, 'a')
            (t, y) = source
            s = '{0}\n{1} {2}\n#\n'.format(t*GRID_T,CHICKEN_X,(y-NUM_GRIDS_BELOW_ZERO)*GRID_Y)
            f.write(s)
            f.close()
            #check Excution
            addBlocks(test_grid, {source[0]:car_info[source[0]]})
            if (test_grid[source[1]][current_time_step+1] != 0):
                print "death!", source, test_grid[source[1]][source[0]]
                death += 1
                source = (current_time_step+1, 0)

            if (source[1] >= NUM_GRIDS_Y - EXTRA_GRIDS):
                cross += 1
                #reach goal
                source = (current_time_step+1, 0)
            current_time_step += 1
        else:
            print 'Offline Source: '+repr(source)
            #car_model.printModel()
            if needUpdateGrid:
                updateGridMap(grid_map, current_time_step, incremental_car_history, car_model, car_model.enough_info)
                needUpdateGrid = False
                #plotGridMap(grid_map)
            came_from, goal  = aStar(grid_map, source)
            generateAnswer(came_from, goal, output_file)
            if (goal[1] >= NUM_GRIDS_Y - EXTRA_GRIDS):
                cross += 1
            source = (goal[0]+1,0)
            current_time_step = source[0]
    #for time_step in range(0, max_time_steps):
    #    if time_step%500 == 0:
    #        print time_step
    #        car_model.printModel()
    #        plotGridMap(grid_map, time_step, time_step + 4)
    getScore(cross, death)
    print 'write to {0} file'.format(output_file)

#overlap of lane configuration space
def getBarrier(car_info):
    grid_map = np.zeros((NUM_GRIDS_Y,HORIZON+len(car_info.keys())))
    #add new history blocks
    for t in range(grid_map.shape[1]):
        #consider car and chicken height
        block_center_grid = np.array([0.05, 0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75 , 0.85, 0.95])/GRID_Y
        for grid_y in block_center_grid:
            for idy in range(int(round(grid_y-HALF_HEIGHT_GRIDS)), int(round(grid_y+HALF_HEIGHT_GRIDS))):
                grid_map[NUM_GRIDS_BELOW_ZERO+idy][t] += 1
    return grid_map

def solver3():
    car_info = loadData(sys.argv[1])
    grid_map = initialEmptyGridMap(car_info)
    barrier = getBarrier(car_info)
    #plotGridMap(barrier, 1000, 1000 + HORIZON)
    test_grid = np.copy(grid_map)
    car_model = CarModel()
    source = (0,0)
    max_time_steps = sorted(car_info.keys())[-1]
    current_time_step = 0
    output_file = sys.argv[3]
    death = 0
    cross = 0
    clearFile(output_file)
    last_valid_came_from = {}
    f = open(output_file, 'a')
    (t, y) = source
    s = '{0}\n{1} {2}\n#\n'.format(0*GRID_T,CHICKEN_X,(0-NUM_GRIDS_BELOW_ZERO)*GRID_Y)
    f.write(s)
    f.close()
    while current_time_step < max_time_steps:
        incremental_car_history = {}
        incremental_car_history[current_time_step] = car_info[current_time_step]
        print 'Online Source: '+repr(source)
        car_model.updateSolver3(incremental_car_history )
        speeds, intervals = car_model.getGaussianParams()
        #print speeds
        #print intervals
        #prin "\n#######################################################################\n"
        #car_model.printModel()
        updateGridMapSolver3(grid_map, current_time_step, incremental_car_history, car_model,speeds, intervals, barrier)
        #if current_time_step % 1000 == 0:
        #    print speeds
        #    print intervals
        #print car_model.initial_pos_xs
        #print car_model.initial_times
        #    plotGridMap(grid_map, current_time_step, current_time_step + 500)
        came_from, sub_goal = oneStepAction(grid_map, source)
        #only need one step and no overlap then just transfer in source
        if sub_goal == (-1,-1):
            goal = last_valid_came_from[sorted(last_valid_came_from.keys())[-1]]
            while goal != None:
                current = last_valid_came_from[goal]
                if current[0] == current_time_step+1:
                    source = current
                    break
                goal = current
        else:
            last_valid_came_from = came_from
            source = sub_goal
        f = open(output_file, 'a')
        (t, y) = source
        s = '{0}\n{1} {2}\n#\n'.format(t*GRID_T,CHICKEN_X,(y-NUM_GRIDS_BELOW_ZERO)*GRID_Y)
        f.write(s)
        f.close()
        #check Excution
        addBlocks(test_grid, {source[0]:car_info[source[0]]})
        if (test_grid[source[1]][current_time_step+1] != 0):
            print "death!", source, test_grid[source[1]][source[0]]
            death += 1
            source = (current_time_step+1, 0)

        if (source[1] >= NUM_GRIDS_Y - EXTRA_GRIDS):
            cross += 1
            #reach goal
            source = (current_time_step+1, 0)
        current_time_step += 1
    f = open(output_file, 'a')
    (t,y) = source
    s = '{0}\n{1} {2}\n#\n'.format(t*GRID_T,CHICKEN_X,(y-NUM_GRIDS_BELOW_ZERO)*GRID_Y)
    f.write(s)
    f.close()

    getScore(cross, death)
    print 'write to {0} file'.format(output_file)


if __name__ == '__main__':
    if len(sys.argv) != 4:
        print 'USAGE:\n python freeway1.py dataXX.txt X(inidate the solver {1,2,3}) outputfileName\n'
        sys.exit(0)
    t0 = time()
    if (sys.argv[2] == '1'):
        solver1()
    elif (sys.argv[2] == '2'):
        solver2()
    elif (sys.argv[2] == '3'):
        solver3()
    t1 = time()
    print 'freeway takes %f' %(t1-t0)
