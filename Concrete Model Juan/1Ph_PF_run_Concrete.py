# ------------------------------------------------------------------------
# Optimal Power Flow Python Code developed by 
# Juan S. Giraldo, TU Eindhoven, j.s.giraldo.chavarriaga@tue.nl
# ------------------------------------------------------------------------
# %% Upload Libraries
from pyomo.environ import *
import One_Ph_PF_Concrete_Model as pf
import math 
import pandas as pd
import numpy

# Importing Data System
System_Data_Nodes = pd.read_excel('Nodes_34.xlsx')
System_Data_Lines = pd.read_excel('Lines_34.xlsx')

# Parameters
Vnom = 11/math.sqrt(3)
Vmin = 0.80*Vnom
Vmax = 1.05*Vnom
      
# Create the Model
model =  pf.create_1_ph_pf_model(Vnom, Vmin, Vmax, System_Data_Nodes, System_Data_Lines)
model.pprint()
# Define the Solver 
solver = SolverFactory('ipopt')
solver.solve(model, tee=True)

losses = value(model.obj)
supply = [sum(value(model.PS[n]) for n in model.NODES)]
demand = [sum(value(model.PD[n]) for n in model.NODES)]
print([supply, demand, losses])