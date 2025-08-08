import numpy as np
import sympy
#import matplotlib.pyplot as plt
import json
import random
from sklearn.preprocessing import OneHotEncoder
import copy
import pandas as pd
from sympy import symbols, Function
import openpyxl


def generate_excel(vehTrajsAtk,item):

    vehIDList= []
    for vehID in vehTrajsAtk.keys():
        vehIDList.append(vehID)

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
    filename='/home/drivesim/Documents/SR/IDMData/'+item+'.xlsx'
    df.to_excel(filename, index=False)  # index=False prevents adding a row index to the file
    return vehIDList

def generate_feature_list(vehTrajs,DatasetID):
    FeatureList=[]
    LatDevList=[]
    WeightList=[]
    for vehID in vehTrajs.keys():
        for i in range(len(vehTrajs[vehID]['featureList'])): #remove the distance as feature 11/11
            #vehTrajs[vehID]['featureList'][i]=vehTrajs[vehID]['featureList'][i]+[vehTrajs[vehID]['firstPhaseDuration']]
            if vehTrajs[vehID]['simTime'][i]>=vehTrajs[vehID]['secondPhaseStartTime']:
                FeatureList.append(vehTrajs[vehID]['featureList'][i])
                LatDevList.append(vehTrajs[vehID]['latdev'][i])
                WeightList.append(vehTrajs[vehID]['weights'][i])
    return FeatureList,LatDevList,WeightList

def one_hot_encode(DatasetList):
    # Create an instance of the OneHotEncoder
    encoder = OneHotEncoder()
    # Fit and transform the data to one-hot encoding
    one_hot_encoded = encoder.fit_transform(np.array(DatasetList).reshape(-1, 1))
    one_hot_encoded = one_hot_encoded.toarray()
    return one_hot_encoded

def moving_average(data, window_size):
    return data.rolling(window=window_size).mean()


def load_trajectory(DatasetID,DatasetList):
    #filename='Highway'+DatasetNum+'FeatureList'+'.json'
    #with open(filename, 'r') as f:    
    #    X_ori = json.load(f)
    #filename='Highway'+DatasetNum+'LatDevList'+'.json'
    #with open(filename, 'r') as f:    
    #    y = json.load(f)
    
    
    filename='/home/drivesim/Documents/SR/TrainDataset/Highway'+DatasetID+'TrajSmooth'+'.json'
    with open(filename,'r') as f:
        vehTrajs=json.load(f)
    TotalTrajNum=110+64+104+19

        #added to PySR
    shortTrajList=[]
    if DatasetID=='36_1' or DatasetID=='36_2' or  DatasetID=='06':
        for vehID in vehTrajs.keys():
            if len(vehTrajs[vehID]['simTime'])<=25:
                shortTrajList.append(vehID)
        for vehID in shortTrajList:
            del vehTrajs[vehID]
    #for vehID in vehTrajs.keys():
    #    if len(vehTrajs[vehID]['simTime'])>80:
    #        print(vehID)
    
    #plot_trajectory(vehTrajs, DatasetID)
    abnormalPatternList=['6767','6794','6808','6766','6844','6765','6845','6599','6803','6800','6802',\
                         '6801','6804']
    if DatasetID=='36_1' or DatasetID=='36_2':
        for vehID in abnormalPatternList:  
            if vehID in vehTrajs.keys():
                del vehTrajs[vehID]
            
    
    #vehTrajs.pop('6797')

    #determine the lane departure threshold
    latdev_threshold=1.2
    for vehID in vehTrajs.keys():
        
        vehTrajs[vehID]['laneDepartTime']=0
        for i in range(len(vehTrajs[vehID]['latdev'])):
            if abs(vehTrajs[vehID]['latdev'][i])>latdev_threshold:
                vehTrajs[vehID]['laneDepartTime']=i/10
                break

        vehTrajs[vehID]['firstPhaseDuration']=vehTrajs[vehID]['secondPhaseStartTime']-vehTrajs[vehID]['firstPhaseStartTime']

          
        
        for i in range(len(vehTrajs[vehID]['featureList'])):
            vehTrajs[vehID]['featureList'][i]=vehTrajs[vehID]['featureList'][i]+[i]
        
        #for time series data
        #vehTrajs[vehID]['featureList'][0].append(vehTrajs[vehID]['latdev'][0])
        #for i in range(1,len(vehTrajs[vehID]['featureList'])):
        #    vehTrajs[vehID]['featureList'][i].append(vehTrajs[vehID]['latdev'][i-1])

        vehTrajs[vehID]['weights']=[]
        for i in range(len(vehTrajs[vehID]['featureList'])):
            if i==len(vehTrajs[vehID]['featureList'])-1:
                vehTrajs[vehID]['weights'].append(1)
            else:
                vehTrajs[vehID]['weights'].append(1)
            '''
            if DatasetID=='06':
                vehTrajs[vehID]['weights'].append(0.68)
            if DatasetID=='17':
                vehTrajs[vehID]['weights'].append(1.152)
            if DatasetID=='36':
                vehTrajs[vehID]['weights'].append(0.735)
            if DatasetID=='37':
                vehTrajs[vehID]['weights'].append(3.731)
            '''
            
    vehTrajsTrain={}
    vehTrajsTest={}
     
    random.seed(77)

    select_num=25
    #if DatasetID=='36_2':
    #    select_num=0

    SelectedList=[]
    SelectedList = random.sample(list(vehTrajs.keys()),  select_num)    #randomly select 25 trajectories for training for each dataset.
    print(len(SelectedList))

    for vehID in vehTrajs.keys():
        vehTrajs[vehID]['DatasetID']=DatasetID
        vehIDNew=DatasetID+'_'+vehID
        if vehID in SelectedList:
            vehTrajsTrain[vehIDNew]=vehTrajs[vehID]
        else:
            vehTrajsTest[vehIDNew]=vehTrajs[vehID]

    
    
    print(DatasetID+':'+str(len(vehTrajsTrain)))
        
    FeatureList,LatDevList,WeightList=generate_feature_list(vehTrajsTrain,DatasetID)
    return FeatureList,LatDevList,WeightList,vehTrajsTest,vehTrajsTrain

#DatasetList=['06','17','36','37'] 
DatasetList=['06','36_1','36_2','37']
vehTrajsTest={} 
vehTrajsTrain={}
FeatureList=[]
WeightList=[]
y=[]

filename = "/home/drivesim/Documents/SR/AllDataset.json"
with open(filename, 'r') as f:
    vehTrajsAll = json.load(f)

for vehID in vehTrajsAll.keys():
    vehTrajsAll[vehID]['distanceAtk']=copy.deepcopy(vehTrajsAll[vehID]['distanceGT'])

vehIDList_speed=generate_excel(vehTrajsAll,'speed')
vehIDList_distanceAtk=generate_excel(vehTrajsAll,'distanceAtk')
vehIDList_accel=generate_excel(vehTrajsAll,'accel')
vehIDList_latdev=generate_excel(vehTrajsAll,'latdev')

vehID_idx_dict={}
k=1
for vehID in vehIDList_speed:
    vehID_idx_dict[vehID]=k
    k=k+1

filename='ID2IDX.json'     #store the training dataset for each trajectory
with open(filename,'w+') as f:
    json.dump(vehID_idx_dict,f)

for DatasetID in DatasetList:
    FeatureListSingle,LatDevListSingle,WeightListSingle,vehTrajsSingle,vehTrajsTrainSingle=load_trajectory(DatasetID,DatasetList)
    
    filename='/home/drivesim/Documents/SR/MSFTrainData/TrainingDataset'+DatasetID+'.json'     #store the training dataset for each trajectory
    with open(filename,'w+') as f:
        json.dump(vehTrajsTrainSingle,f)
    for vehID in vehTrajsSingle.keys():
        #vehIDNew=DatasetID+'_'+vehID    
        vehTrajsTest[vehID]=vehTrajsSingle[vehID]    
    for vehID in vehTrajsTrainSingle.keys():
        #vehIDNew=DatasetID+'_'+vehID
        vehTrajsTrain[vehID]=vehTrajsTrainSingle[vehID]

    FeatureList+=FeatureListSingle
    WeightList+=WeightListSingle
    y+=LatDevListSingle

X=np.zeros((len(FeatureList),len(FeatureList[0])))
for i in range(len(FeatureList)):
    for j in range(len(FeatureList[0])):
        X[i,j]=FeatureList[i][j]

vehIDDict={}        
for vehID in vehTrajsTrain.keys():
    if vehTrajsTrain[vehID]['DatasetID'] not in vehIDDict.keys():
        vehIDDict[vehTrajsTrain[vehID]['DatasetID']]=[]
    vehIDDict[vehTrajsTrain[vehID]['DatasetID']].append(vehID)

#print(vehIDDict)

vehTrajsTrainSimple={}
for vehID in vehTrajsTrain.keys():
    singleDict={"featureList":[],"latdev":[]}
    singleDict["featureList"]=vehTrajsTrain[vehID]["featureList"]
    singleDict["latdev"]=vehTrajsTrain[vehID]["latdev"]
    vehTrajsTrainSimple[vehID]=singleDict


filename='/home/drivesim/Documents/SR/MSFTrainData/TrainingDatasetSimple'+'.json' 
with open(filename,'w+') as f:
    json.dump(vehTrajsTrainSimple,f)

filename='/home/drivesim/Documents/SR/MSFTrainData/TrainingDataset'+'.json' 
with open(filename,'w+') as f:
    json.dump(vehTrajsTrain,f)

filename='/home/drivesim/Documents/SR/MSFTrainData/TrainingID'+'.json'
with open(filename,'w+') as f:
    json.dump(vehIDDict,f)


#filename='vehTrajsTrain'+'.json' 
#with open(filename,'w+') as f:
#    json.dump(vehTrajsTrain,f)


filename='vehTrajsTestNCR77'+'.json'
with open(filename,'w+') as f:
    json.dump(vehTrajsTest,f)


print("start model")

import juliapkg

juliapkg.add("JSON", "682c06a0-de6a-54ab-a142-c8b1cf79cde6",dev=False)

from pysr import PySRRegressor, TemplateExpressionSpec
import pysr
print(pysr.__version__)



print("import PySRRegressor")

#nested_constraints={
#                    "exp": {"exp":0,"sin":0,"log":0,"^":0},
#                    "sin": {"sin":0,"log":0,"exp":0,"^":0},
#                    "log": {"exp":0,"sin":0,"log":0,"^":0},
#                    "^": {"exp":0,"sin":0,"log":0,"^":0}
#},



template = TemplateExpressionSpec(
    function_symbols=["f", "g", "h"],
    combine="((; f, g, h), (x1, x2, x3, x4, x5, x6, x7, x8)) -> h(x1,x2,x3,x4,x5,x6,x7,x8)*exp(f(x1,x2,x3,x4,x5,x6,x7,x8)) + g(x1,x2,x3,x4,x5,x6,x7,x8)",
)


model = PySRRegressor(
    niterations=100,  # < Increase me for better results
    expression_spec=template,
    binary_operators=["*", "+", "-"],
    #unary_operators=[
    #   "log", "exp","sin"
    #],

    #extra_sympy_mappings={"inv": lambda x: 1 / x},
    maxsize=35,
    maxdepth=10,
    #constraints={
    #    "/": (-1, 3),
    #    "exp": 3,
    #
    #},
    #nested_constraints={
    #                "exp": {"exp":0,"log":0,"^":0},
    #                "log": {"exp":0,"log":0,"^":0},
    #                "^": {"exp":0,"log":0,"^":0}
    #},
    #nested_constraints={
    #                "exp": {"exp":0,"sin":0,"log":0,"^":0},
    #                "sin": {"sin":0,"log":0,"exp":0,"^":0},
    #                "log": {"exp":0,"sin":0,"log":0,"^":0},
    #                "^": {"exp":0,"sin":0,"log":0,"^":0}
    #},
    #select_k_features=7,
    #constraints={"pow": (9, 1)},
    model_selection='accuracy',
    # ^ Define operator for SymPy as well
    #weights=WeightList,
    #denoise=True,b
    #loss="loss(prediction, target) = (prediction - target)^2",
    random_state=77,  #
    procs = 0,
    deterministic = True,
    fraction_replaced = 0.03,
    parsimony = 0.0025,
    population_size=200,
    parallelism='serial'
    #warm_start=True,


    
    #elementwise_loss="loss(prediction, target, weight) = weight*(prediction - target)^2",
    # ^ Custom loss function (julia syntax)
)


#model.fit(X, y,weights=WeightList)

print("start model fitting")
model.fit(X, y)

print(model)

from scipy.optimize import curve_fit

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

params, covariance = curve_fit(model_opt, model.predict(X), y, p0=[2, 1])  # Initial guesses for a and b

# Extract fitted parameters
a, b = params
print(f"Fitted parameters: a = {a}, b = {b}")

# Predicted values
#y_pred = model(X, a, b)
#print(f"Predicted y values: {y_pred}")


for vehID in vehTrajsTest.keys():
    vehTrajsTest[vehID]['y_pred']=[]
    if len(vehTrajsTest[vehID]['featureList'])!=0:
            for i in range(len(vehTrajsTest[vehID]['featureList'])):
                X_test = np.zeros((1,len(vehTrajsTest[vehID]['featureList'][0])))
                #print(len(vehTrajsTest[vehID]['featureList'][0]))
                for j in range(len(vehTrajsTest[vehID]['featureList'][0])): 
                    X_test[0,j]=vehTrajsTest[vehID]['featureList'][i][j]
                    #if i!=0:
                    #    X_test[0,-1] = vehTrajsTest[vehID]['y_pred'][-1]
                y_pred=model.predict(X_test)
                vehTrajsTest[vehID]['y_pred'].append(y_pred[0])




def calculate_testing_loss(vehTrajs):
    mseList=[]
    for vehID in vehTrajs.keys():
        if len(vehTrajs[vehID]['y_pred'])!=0:
            mse=0
            for i in range(len(vehTrajs[vehID]['y_pred'])):
                mse+=abs(vehTrajs[vehID]['y_pred'][i]-vehTrajs[vehID]['latdev'][i])
            mse=mse/len(vehTrajs[vehID]['y_pred'])
            mseList.append(mse)
    return mseList

mseList=calculate_testing_loss(vehTrajsTest)
meanMse=np.mean(mseList)  
print(meanMse)


'''
for vehID in vehTrajsTest.keys():
    plt.figure()
    xpoints=range(len(vehTrajsTest[vehID]['y_pred']))
    ypoints=vehTrajsTest[vehID]['y_pred']
    plt.plot(xpoints,ypoints,color='green',label='predict curve')
    ypoints=vehTrajsTest[vehID]['latdev']
    plt.plot(xpoints,ypoints,color='red',label='original curve')
    plt.xlabel('simulation time (0.1s)')
    plt.ylabel('lateral deviation (m)')
    plt.title(vehTrajsTest[vehID]['DatasetID']+': '+str(vehID))
    plt.legend()
    plt.show()
'''   
          

