import json
import numpy as np
import matplotlib.pyplot as plt

'''
filename='/home/perception/Jun/MODELINGATTACKTRAJECTORY/PYSR_Traj_CR.json' 
with open(filename, 'r') as f:
    vehTrajsAtk = json.load(f)
    

filename='/home/perception/Jun/MODELINGATTACKTRAJECTORY/PYSR_Traj_NCR.json' 
with open(filename, 'r') as f:
    vehTrajsAtkNCR = json.load(f)
''' 
    
filename='/home/drivesim/Documents/SR/31PYSRTest77NCR.json' #with crash rate
with open(filename, 'r') as f:
    vehTrajsAtkNCR = json.load(f)  

#filename='/home/drivesim/Documents/SR/ID2IDX.json' #with crash rate
#with open(filename, 'r') as f:
#    vehIDDict = json.load(f)  

filename='/home/drivesim/Documents/SR/31PYSRTest77NCR.json' #with crash rate
with open(filename, 'r') as f:
    vehTrajsAtk = json.load(f)  
    
#filename='/home/drivesim/Documents/SR/TrainDataset/Highway'+'06'+'TrajSmooth'+'.json'
#with open(filename,'r') as f:
#    vehTrajs=json.load(f)
    
# calculate max vehicle speed and accel
def calculate_max_speed_accel(vehTrajs):
    for vehID in vehTrajs.keys():
        vehTrajs[vehID]['max_accel']=[]
        vehTrajs[vehID]['max_speed']=[]
        for i in range(len(vehTrajs[vehID]['simTime'])):
            max_accel=max(vehTrajs[vehID]['accel'][0:i])
            min_accel=min(vehTrajs[vehID]['accel'][0:i])
            vehTrajs[vehID]['max_accel'].append(max(abs(max_accel),abs(min_accel)))
            vehTrajs[vehID]['max_speed'].append(max(vehTrajs[vehID]['speed'][0:i]))
    return vehTrajs

#def calculate_psi_rate(vehTrajs)

for vehID in vehTrajsAtk.keys():
    plt.figure()
    datasetID=vehTrajsAtk[vehID]['DatasetID']
    xpoints=range(len(vehTrajsAtk[vehID]['latdev']))
    ypoints=[]
    for i in xpoints:
        ypoints.append(abs(vehTrajsAtk[vehID]['latdev'][i]))
    #plt.plot(xpoints,ypoints,color='green',label='MSF attack lateral deviation CR')
    ypoints=[]
    for i in xpoints:
        ypoints.append(abs(vehTrajsAtk[vehID]['latdevMSF'][i]))
    plt.plot(xpoints,ypoints,color='red',label='MSF attack lateral deviation ')
    ypoints=[]
    for i in xpoints:
        ypoints.append(abs(vehTrajsAtkNCR[vehID]['latdev'][i]))
    plt.plot(xpoints,ypoints,color='black',label='TI-SR lateral deviation')   
    plt.xlabel('simulation time (0.1s)')
    plt.ylabel('lateral deviation (m)')
    #plt.title(vehTrajsTest[vehID]['DatasetID']+': '+str(vehID))
    plt.legend()
    plt.show()
    #save_path='/home/perception/Jun/MODELINGATTACKTRAJECTORY/KAIST FIGURES/PYSRTRAJ/'+str(datasetID)+str(vehID)+'.png'
    #plt.savefig(save_path)
