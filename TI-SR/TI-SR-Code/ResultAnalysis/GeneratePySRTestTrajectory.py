import json
import numpy as np
import matplotlib.pyplot as plt
import random
import math
#from pysr import PySRRegressor
from pysr import PySRRegressor, TemplateExpressionSpec
import copy

#from dso import DeepSymbolicOptimizer
from sklearn.preprocessing import OneHotEncoder
#from dso import DeepSymbolicRegressor

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
from scipy.special import erf

#from constants import *
#from pyomo.environ import *
#from pyomo.opt import SolverFactory
#from pyomo.opt import SolverStatus, TerminationCondition
import logging
from numpy import linalg as LA

#from pyomo.util.infeasible import log_infeasible_constraints     
import json
from sklearn.model_selection import train_test_split
import time
#from pyomo.util.model_size import build_model_size_report
#from pyomo.util.infeasible import log_infeasible_constraints

import multiprocessing
import socket
import struct
import seaborn as sns

import pandas as pd

#from DeterministicTestFunctions import calculate_lateral_deviation_PySR

#calculate mse
def calculate_testing_loss(vehTrajs):
    mseList=[]
    for vehID in vehTrajs.keys():
        if len(vehTrajs[vehID]['y_pred'])!=0:
            mse=0
            for i in range(len(vehTrajs[vehID]['y_pred'])):
                mse+=abs(vehTrajs[vehID]['y_pred'][i]-vehTrajs[vehID]['latdevMSF'][i])
            mse=mse/len(vehTrajs[vehID]['y_pred'])
            mseList.append(mse)
    return mseList

def generate_excel(vehTrajsAtk,item,CRFlag,complexity):
    
    singleDict={}
    for vehID in vehTrajsAtk.keys():
        singleDict[vehID]=copy.deepcopy(vehTrajsAtk[vehID][item])
    
    lenList=[]
    for vehID in singleDict.keys():
        lenList.append(len(singleDict[vehID]))
    maxLen=max(lenList)
    
    for vehID in singleDict.keys():
        idx=maxLen-len(singleDict[vehID])
        for i in range(idx):
            singleDict[vehID].append(10000)
            
    # Convert the dictionary to a pandas DataFrame        
    df = pd.DataFrame(singleDict)   
    
    # Write the DataFrame to an Excel file
    if CRFlag==1:
        filename='/home/drivesim/Documents/SR/PySRData/'+str(complexity)+item+'PySR.xlsx'
    else:
        filename='/home/drivesim/Documents/SR/PySRData/'+str(complexity)+item+'PySRNCR.xlsx'
    df.to_excel(filename, index=False)  # index=False prevents adding a row index to the file


#generate trajectory using PySR model

filename = "/home/drivesim/Documents/SR/SymbolicRegression.jl/src/vehTrajsTestNCR00.json" #vehTrajsTestCRHD  #change random seed
#filename="/home/perception/Jun/MODELINGATTACKTRAJECTORY/TrainingResult/vehTrajsTestCR.json"
with open(filename, 'r') as f:
    vehTrajsAtk = json.load(f)

#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-09-18_162441.532.pkl") #with CR and HD matrix
#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-09-19_171846.146.pkl") #with CR and HD matrix
#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-09-23_152239.617.pkl") #with CR matrix deterministic test random seed 77

#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-09-24_221245.978.pkl") #no CR matrix deterministic random seed 77
#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-09-30_135903.621.pkl")  #CR and lane departure time deterministic test

#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-10-01_165548.428.pkl") #PySR with CR and complexity
#print(model)
#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-10-03_133521.853.pkl") #PySR , complexity
#print(model)

#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-10-09_104417.493.pkl") #PySR  CR complexity parsimony 0.001
#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-10-10_113446.256.pkl") #PySR without CR parsimony 0.001 best
#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-10-10_150147.406.pkl") #PySR without CR accuracy

#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-10-16_155419.948.pkl") #1000 iteration 20 max complexity 
#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-10-20_165921.916.pkl") #500 iteration 20 max complexity traffic informed in the last 100 iteration

#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-10-22_101758.313.pkl") #500 iteration 20 max complexity
#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-10-22_143606.261.pkl") #200 iteration 20 max complexity with CR
#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-10-25_194004.729.pkl") #200 iteration 20 max complexity

# above model use random seed 77
#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-10-25_214635.813.pkl") # 100 iteration 30 max complexity, traffic informed random seed 66
#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-10-28_141854.157.pkl") # 200 iteration 35 max complexity, traffic informed, random seed 11, bilevel selection 
#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-10-29_141522.378.pkl") #200 iteration 35 max complexity, without traffic informed, random seed 11

#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-10-30_123057.307.pkl") # 500 iteration 35 max complexity, without traffic informed, random seed 11
#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-11-03_103125.375.pkl")  # 200 iteration 35 max complexity, no traffic informed, weight 100 for last item, random seed 11

#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-11-03_105808.629.pkl") # 200 iteration 35 max complexity, no traffic informed, weight 1000 for last item, random seed 11
#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-11-03_113909.486.pkl") # 200 iteration 35 max complexity. traffic informed,weight 1000 for last item, random seed 11

#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-11-07_144658.974.pkl") #500 iteration 35 max complexity, traffic informed, random seed 77
#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-11-10_222616.727.pkl") # 500 iteration 35 max complexity, no traffic informed, random seed 77

#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-11-16_215009.783.pkl") #200 iteration, 35 max complexity, traffic informed, random seed 77, include sin
#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-11-19_105510.909.pkl") # 200 iteration, 35 max complexity, no traffic informed
#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-11-20_002554.719.pkl") # 100 iteration, 35 max complexity, traffic informed, random seed 77, include sin, 500 population size

#model = PySRRegressor.from_file("/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRModel_CR_HD/hall_of_fame_2024-11-19_142542.622.pkl") #200 iteration, 35 max complexity, no traffic informed, random seed 77, 500 population

#model = PySRRegressor.from_file(run_directory="/home/drivesim/Documents/SR/SymbolicRegression.jl/src/outputs/20250118_203801_DqDZtF/") # 100 iteration, 35 max complexity, traffic informed, fix function form
#model = PySRRegressor.from_file(run_directory="/home/drivesim/Documents/SR/SymbolicRegression.jl/src/outputs/20250120_144437_eNjK7p/") 

#new results with function form y=f(x)*e^g(x)+h(x)
#model = PySRRegressor.from_file(run_directory="/home/drivesim/Documents/SR/SymbolicRegression.jl/src/outputs/20250121_205220_2RVPrC/")  #100 iteration, 35 max complexity, traffic informed, f(x)*e^g(x)+h(x), random state 77, best result
#model = PySRRegressor.from_file(run_directory="/home/drivesim/Documents/SR/SymbolicRegression.jl/src/outputs/20250122_161443_IaCnvT/") #100 iteration, 35 max complexity, no traffic informed, f(x)*e^g(x)+h(x), random state 77
#model = PySRRegressor.from_file(run_directory="/home/drivesim/Documents/SR/SymbolicRegression.jl/src/outputs/20250127_133447_dv9qKI/") #100 iteration, 35 max complexity, traffic informed, random seed 55
#model = PySRRegressor.from_file(run_directory="/home/drivesim/Documents/SR/SymbolicRegression.jl/src/outputs/20250128_100359_lvcKtc/")  #100 iteration, 35 max complexity, no traffic informed, random seed 55
#model = PySRRegressor.from_file(run_directory="/home/drivesim/Documents/SR/SymbolicRegression.jl/src/outputs/20250125_204456_SpNHVp/") #100 iteration, 35 max complexity, no traffic informed, random seed 66
#model = PySRRegressor.from_file(run_directory="/home/drivesim/Documents/SR/SymbolicRegression.jl/src/outputs/20250128_130256_AOXD4h/") #100 iteration, 35 max complexity, traffic informed, random seed 66
#model = PySRRegressor.from_file(run_directory="/home/drivesim/Documents/SR/SymbolicRegression.jl/src/outputs/20250129_143716_pTuxNI/") #no traffic informed, random seed 11
#model = PySRRegressor.from_file(run_directory="/home/drivesim/Documents/SR/SymbolicRegression.jl/src/outputs/20250131_001758_xsZxVx/") #no traffic informed, random seed 11, not using KA06
#model = PySRRegressor.from_file(run_directory="/home/drivesim/Documents/SR/SymbolicRegression.jl/src/outputs/20250130_225851_vAPVDL/") # no traffic informed, not using KA37
#model = PySRRegressor.from_file(run_directory="/home/drivesim/Documents/SR/SymbolicRegression.jl/src/outputs/20250131_103755_BbsX63/") # no traffic informed, not using KA36_1
#
#model = PySRRegressor.from_file(run_directory="/home/drivesim/Documents/SR/SymbolicRegression.jl/src/outputs/20250207_001037_zTEvvb/") # traffic informed, not using KA06, random seed 00
model = PySRRegressor.from_file(run_directory="/home/drivesim/Documents/SR/SymbolicRegression.jl/src/outputs/20250210_151622_WouM8q/") # traffic informed, random seed 00, 200 population, 
#model = PySRRegressor.from_file(run_directory="/home/drivesim/Documents/SR/SymbolicRegression.jl/src/outputs/20250217_135536_qIf3t2/") #no traffic informed, random seed 00, 200 population


a = -1.471609939618567
b = 0.33449656028616975

def model_opt(x, a, b):
    """
    Computes the model: a * exp(x) + b element-wise for the input x.
#
    Parameters:
    x (array-like): Input data (can be a scalar, list, or NumPy array).
    a (float or array-like): Coefficient for the exponential term.
    b (float or array-like): Constant term.

    Returns:
    numpy.ndarray: Result of the computation.
    """
    return a * np.exp(x) + b


def calculate_lateral_deviation_PySR(vehTrajs,model,best_index):
    for vehID in vehTrajs.keys():
        vehTrajs[vehID]['y_pred']=[]
        
        x_test=np.zeros((len(vehTrajs[vehID]['featureList']),\
                         len(vehTrajs[vehID]['featureList'][0]))) #7 features
        
        k=0
        for i in range(len(vehTrajs[vehID]['featureList'])):
            for j in range(len(vehTrajs[vehID]['featureList'][0])):  #7 features
                x_test[k,j]=vehTrajs[vehID]['featureList'][i][j]
            k+=1
        #vehTrajs[vehID]['x_test']=x_test
        y_pred=model.predict(x_test, index=best_index)
            
        #y_pred_fix = model_opt(y_pred, a, b)
        #for i in range(vehTrajs[vehID]['firstPhaseDuration']):
        #    vehTrajs[vehID]['y_pred'].append(vehTrajs[vehID]['latdev'][i])
        vehTrajs[vehID]['y_pred']+=list(y_pred)
        
        '''
        vehTrajsTest[vehID]['y_pred']=[]
        if len(vehTrajsTest[vehID]['featureList'])!=0:
            for i in range(len(vehTrajsTest[vehID]['featureList'])):
                X_test = np.zeros((1,len(vehTrajsTest[vehID]['featureList'][0])))
                #print(len(vehTrajsTest[vehID]['featureList'][0]))
                for j in range(len(vehTrajsTest[vehID]['featureList'][0])): 
                    X_test[0,j]=vehTrajsTest[vehID]['featureList'][i][j]
                    if i!=0:
                        X_test[0,-1] = vehTrajsTest[vehID]['y_pred'][-1]
                y_pred=model.predict(X_test)
                vehTrajsTest[vehID]['y_pred'].append(y_pred[0])
        '''
        #print(len(vehTrajsTest[vehID]['y_pred']))
    return vehTrajs

def generate_complexity_file(model,CRFlag):
        # Initialize an empty dictionary and list
    singleDict = {}
    complexity_list = []
    
    # Extract complexities from the model
    for i in range(len(model)):
        complexity_list.append(model["complexity"][i])  
    
    # Create a DataFrame from the complexity list
    df = pd.DataFrame(complexity_list, columns=["Complexity"])

    # Set the filename based on CRFlag
    if CRFlag == 1: 
        filename = '/home/drivesim/Documents/SR/PySRData/ComplexityPySR.xlsx'
    else:
        filename = '/home/drivesim/Documents/SR/PySRData/ComplexityPySRNCR.xlsx'
    
    # Deep copy the list into the dictionary (if needed for further use)
    singleDict["complexity_list"] = copy.deepcopy(complexity_list)
    
    # Write the DataFrame to an Excel file
    df.to_excel(filename, index=False)


def generate_Mse_file(CRFlag,meanMseList):
    
    
    # Create a DataFrame from the complexity list
    df = pd.DataFrame(meanMseList, columns=["Complexity"])

    if CRFlag == 1: 
        filename = '/home/drivesim/Documents/SR/PySRData/MsePySR.xlsx'
    else:
        filename = '/home/drivesim/Documents/SR/PySRData/MsePySRNCR.xlsx'

    df.to_excel(filename, index=False)
    # Deep copy the list into the dictionary (if needed for further use)



print(model)
pareto_models = model.equations_
hof_models = model.get_hof()  # This will include both Pareto and non-Pareto models

print(hof_models)
# Select the model with the lowest loss
best_loss_model = hof_models.loc[hof_models["loss"].idxmin()]
# Get the index of that model
best_loss_index = hof_models["loss"].idxmin()
# Use it for prediction


meanMseList=[]
print(len(hof_models))
for i in range(14,15):
    best_loss_index = i
    complexity = hof_models["complexity"][i]

    with open(filename, 'r') as f:
        vehTrajsAtk = json.load(f)
    
    vehTrajsAtk=calculate_lateral_deviation_PySR(vehTrajsAtk,model,best_loss_index)

    
    for vehID in vehTrajsAtk.keys():
        vehTrajsAtk[vehID]['latdevMSF']=copy.deepcopy(vehTrajsAtk[vehID]['latdev'])    
        vehTrajsAtk[vehID]['latdev']=copy.deepcopy(vehTrajsAtk[vehID]['y_pred'])
        for j in range(len(vehTrajsAtk[vehID]['latdev'])):
            vehTrajsAtk[vehID]['latdev'][j]= float(vehTrajsAtk[vehID]['latdev'][j])
        if vehTrajsAtk[vehID]['latdevMSF']==vehTrajsAtk[vehID]['latdev']:
            print(vehID)
        #for item in vehTrajsAtk[vehID].keys():
        #    if isinstance(vehTrajsAtk[vehID][item][0], np.float32):
        #        print(item)

    vehTrajsAtkSimple={}
    for vehID in vehTrajsAtk.keys():
        singleTrajs={"latdev":[],"latdevMSF":[],"DatasetID":10000}
        singleTrajs["latdevMSF"]=copy.deepcopy(vehTrajsAtk[vehID]['latdevMSF'])
        singleTrajs["latdev"]=copy.deepcopy(vehTrajsAtk[vehID]['latdev'])
        singleTrajs["DatasetID"]=copy.deepcopy(vehTrajsAtk[vehID]['DatasetID'])
        vehTrajsAtkSimple[vehID]=singleTrajs

        

    mseList=calculate_testing_loss(vehTrajsAtk)
    meanMse=np.mean(mseList)  
    print("complexity: "+str(complexity)+','+'meanMSE: '+ str(meanMse))
    print(meanMse)
    meanMseList.append(meanMse)

    filename='/home/drivesim/Documents/SR/'+str(complexity)+'PYSRTest77NCR.json' 
    with open(filename,'w+') as f:
         json.dump(vehTrajsAtkSimple,f)

    for vehID in vehTrajsAtk.keys():
        vehTrajsAtk[vehID]['distanceAtk']=copy.deepcopy(vehTrajsAtk[vehID]['distanceGT'])

    #generate_excel(vehTrajsAtk,'speed',1, complexity)
    #generate_excel(vehTrajsAtk,'distanceAtk',1, complexity)
    #generate_excel(vehTrajsAtk,'accel',1, complexity)
    #generate_excel(vehTrajsAtk,'latdev',1, complexity)
    #generate_excel(vehTrajsAtk,'latdevMSF',1, complexity) #generate the MSF lateral deviation for testing trajectory

#generate_complexity_file(hof_models,1)
#generate_Mse_file(1,meanMseList) 