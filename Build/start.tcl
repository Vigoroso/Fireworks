system partSys dim 1000 0 3 0
simulator partSim link partSys 1000
simulator partSim integration verlet .001
simulator partSim gravity -9.8
simulator partSim drag 0.01