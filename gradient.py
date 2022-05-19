import pybullet as pybullet
import numpy as np
import time
from utils_simulation import *
from nn_stochastic_controller import nn_stochastic_controller
from nn_stochastic_controller import *

'''
def compute_gradient(self, n_E,params, husky, sphere,controller):
    GUI=False
    T_horizon = params['T_horizon']

    np.random.seed(None)
    index = np.random.randint(0,1000000,size=n_E)
    time_list=np.append([0],np.random.randint(0, high=T_horizon, size=10))

    inputs=0
    deltas=0

    for E in range(n_E):
        td_list=[]
        seed_E=index[E]
        eta_list=[]
        input_list=[]

        cost=single_environment_cost(controller, params, husky, sphere, GUI, seed_E)
        for t in range(T_horizon):
            if t in time_list:
                td=(np.random.random()-0.5)*0.4
                td_list.append([td])
                cost_shift,y_t=shift_environment_cost(controller,td, params, husky, sphere, t, seed_E)
                eta_list.append(cost_shift)
                input_list.append(y_t)

            delta_list=(np.array(eta_list)-cost)/np.array(td_list)
            input_array=np.array(input_list)
            Q_array=np.array(delta_list)
            #print(input_array.shape)

            if isinstance(inputs,int):
                inputs=input_array
                deltas=Q_array
            else:
                inputs=np.vstack((inputs, input_array))
                deltas=np.vstack((deltas, Q_array))
        #print(inputs.shape)
        self.z= self.controller.compute_gradient(inputs,deltas)
        self.controller.update1(inputs,deltas)
        return self.z
'''

def compute_gradient(n_E,params, husky, sphere,controller,popula=30):
    index = np.random.randint(0,1000000,size=popula)
    index1= np.random.randint(0,10000)
    Q_positive=0
    Q_negtive=0
    grad1_list=[]
    grad2_list=[]
    q=0.0
    for i in range(popula):
        seed_i=index[i]
        controller.set_random_para(seed_i)
        Q_positive,_=environment_costs(n_E, controller, params, husky, sphere, False, index1, 1)
        Q_negtive,_=environment_costs(n_E, controller, params, husky, sphere, False, index1, 2)
        grad1,grad2= controller.compute_gradient(Q_positive,Q_negtive)
        grad1_list.append(grad1)
        grad2_list.append(grad2)
        q=q+Q_positive+Q_negtive
    
    grad1_fin=averge_gra(grad1_list)
    grad2_fin=averge_gra(grad2_list)
    #print(grad1_fin)
    #print(grad2_fin)
    print(controller.params)
    print(controller.params2)
    controller.update(grad1_fin,grad2_fin)
    #print(q/2.0/popula)
    return 

if __name__=='__main__':
    a=1


