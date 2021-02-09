# ------------------------------------------------------------------------
# Optimal Power Flow Python Code developed by 
# Juan S. Giraldo, TU Eindhoven, j.s.giraldo.chavarriaga@tue.nl
# ------------------------------------------------------------------------
from pyomo.environ import *

def create_1_ph_pf_model(Vnom, Vmin, Vmax, System_Data_Nodes, System_Data_Lines):
    # Network Data
    NODES = [System_Data_Nodes.loc[i,'NODES'] for i in System_Data_Nodes.index]
    Tb = {NODES[i]: System_Data_Nodes.loc[i,'Tb'] for i in System_Data_Nodes.index}
    PD = {NODES[i]: System_Data_Nodes.loc[i,'PD'] for i in System_Data_Nodes.index}      
    QD = {NODES[i]: System_Data_Nodes.loc[i,'QD'] for i in System_Data_Nodes.index} 
    LINES = {(System_Data_Lines.loc[i,'FROM'],System_Data_Lines.loc[i,'TO']) for i in System_Data_Lines.index}
    R = {(System_Data_Lines.loc[i,'FROM'],System_Data_Lines.loc[i,'TO']):System_Data_Lines.loc[i,'R'] for i in System_Data_Lines.index}
    X = {(System_Data_Lines.loc[i,'FROM'],System_Data_Lines.loc[i,'TO']):System_Data_Lines.loc[i,'X'] for i in System_Data_Lines.index}
       
    # Type of Model
    model = ConcreteModel()
    
    # Define Sets
    model.NODES = Set(initialize=NODES)
    model.LINES = Set(initialize=LINES)
    
    # Define Parameters
    model.Vnom = Param(initialize=Vnom, mutable=True)
    model.Vmin = Param(initialize=Vmin, mutable=True)
    model.Vmax = Param(initialize=Vmax, mutable=True)
    model.Tb = Param(model.NODES, initialize=Tb, mutable=True)
    model.PD = Param(model.NODES, initialize=PD, mutable=True) # Node demand
    model.QD = Param(model.NODES, initialize=QD, mutable=True) # Node demand
    model.R = Param(model.LINES, initialize=R, mutable=True) # Line resistance
    model.X = Param(model.LINES, initialize=X, mutable=True) # Line resistance
    
    def R_init_rule(model, i,j):
        return (model.R[i,j]/1000)
    model.RM = Param(model.LINES, initialize= R_init_rule) # Line resistance
    
    def X_init_rule(model, i,j):
        return (model.X[i,j]/1000)
    model.XM = Param(model.LINES, initialize= X_init_rule) # Line resistance
    
    # Define Variables
    model.P = Var(model.LINES, initialize=0) # Acive power flowing in lines
    model.Q = Var(model.LINES, initialize=0) # Reacive power flowing in lines
    model.I  = Var(model.LINES, initialize=0) # Current of lines
    
    def PS_init_rule(model, i):
        if model.Tb[i] == 0:
            temp = 0.0
            model.PS[i].fixed = True
        else:
            temp = 0.0
        return temp
    model.PS = Var(model.NODES, initialize = PS_init_rule)  # Active power of the SS
    
    def QS_init_rule(model, i):
        if model.Tb[i] == 0:
            temp = 0.0
            model.QS[i].fixed = True
        else:
            temp = 0.0
        return temp
    model.QS = Var(model.NODES, initialize = QS_init_rule)  # Active power of the SS
    
    # Voltafe of nodes
    def Voltage_init(model, i):
        if model.Tb[i] == 1:
            temp = model.Vnom
            model.V[i].fixed = True
        else:
            temp = model.Vnom
            model.V[i].fixed = False
        return temp
    model.V = Var(model.NODES, initialize = Voltage_init)
    
    #%% Define Objective Function
    def act_loss(model):
        return (sum(model.RM[i,j]*(model.I[i,j]**2) for i,j in model.LINES))
    model.obj = Objective(rule=act_loss)
    
    #%% Define Constraints
    def active_power_flow_rule(model, k):
            return (sum(model.P[j,i] for j,i in model.LINES if i == k ) - sum(model.P[i,j] + model.RM[i,j]*(model.I[i,j]**2) for i,j in model.LINES if k == i) + model.PS[k] == model.PD[k])
    model.active_power_flow = Constraint(model.NODES, rule=active_power_flow_rule)
    
    def reactive_power_flow_rule(model, k):
            return (sum(model.Q[j,i] for j,i in model.LINES if i == k ) - sum(model.Q[i,j] + model.XM[i,j]*(model.I[i,j]**2) for i,j in model.LINES if k == i) + model.QS[k] == model.QD[k])
    model.reactive_power_flow = Constraint(model.NODES, rule=reactive_power_flow_rule)
    
    def voltage_drop_rule(model, i, j):
        return (model.V[i]**2 - 2*(model.RM[i,j]*model.P[i,j] + model.XM[i,j]*model.Q[i,j]) - (model.RM[i,j]**2 + model.XM[i,j]**2)*model.I[i,j]**2 - model.V[j]**2 == 0)
    model.voltage_drop = Constraint(model.LINES, rule=voltage_drop_rule)
    
    def define_current_rule (model, i, j):
        return ((model.I[i,j]**2)*(model.V[j]**2) == model.P[i,j]**2 + model.Q[i,j]**2)
    model.define_current = Constraint(model.LINES, rule=define_current_rule)
    
    def current_limit_rule(model, i,j):
        return (0, model.I[i,j], None)
    model.current_limit = Constraint(model.LINES, rule=current_limit_rule)
    
    def voltage_limit_rule(model, i):
        return (model.Vmin,model.V[i],model.Vmax)
    model.voltage_limit = Constraint(model.NODES, rule=voltage_limit_rule)
    
    return model

