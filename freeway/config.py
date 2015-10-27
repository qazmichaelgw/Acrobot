import math
import numpy as np
GRID_T = 0.02
GRID_Y = 0.0025
CHICKEN_X = 0.5
CHICKEN_W = 0.04375
CHICKEN_H = 0.05
CHICKEN_MAX_SPEED = 0.0125#by timestep

CAR_W = 0.0475
CAR_H = 0.065

INITIAL_X = CHICKEN_X
INITIAL_Y = -0.0625
GOAL_Y = 1.025

EXTRA_GRIDS = 20
NUM_GRIDS_BELOW_ZERO = int(round(abs(INITIAL_Y/GRID_Y)))
NUM_GRIDS_Y = int(round(GOAL_Y/GRID_Y)) + NUM_GRIDS_BELOW_ZERO + EXTRA_GRIDS
BLOCK_MIN_T = CHICKEN_X - CHICKEN_W/2 - CAR_W/2
BLOCK_MAX_T = CHICKEN_X + CHICKEN_W/2 + CAR_W/2
HALF_HEIGHT_GRIDS =  int(round((CHICKEN_H/2+CAR_H/2)/GRID_Y))

#config for car model
NUM_LANES = 10
CAR_MIN_SPEED = 0.002
CAR_MAX_SPEED = 0.03

#online planning
HORIZON = 33
SAMPLES = 8000
PHI = 0.95#horizon discount factor
PHI_TABLE = np.ones(HORIZON)
for i in range(0,PHI_TABLE.shape[0]-1):
    PHI_TABLE[i+1] = PHI_TABLE[i]*PHI
COEFF = 1
EXP_COEFF = np.exp(COEFF)
#print PHI_TABLE