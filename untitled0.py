import numpy as np
import matplotlib.pyplot as plt

# IDM parameters
v0 = 30.0         # desired velocity (m/s)
T = 1.5           # safe time headway (s)
a_max = 1.0       # maximum acceleration (m/s^2)
b = 1.5           # comfortable deceleration (m/s^2)
delta = 4         # acceleration exponent
s0 = 2.0          # minimum gap (m)





# IDM acceleration function
def idm_acceleration(s, v, delta_v):
    s_star = s0 + v * T + (v * delta_v) / (2 * np.sqrt(a_max * b))
    acc = a_max * (1 - (v / v0) ** delta - (s_star / s) ** 2)

    return acc

def idm_acceleration_under_attack(s, v, delta_v):
    s_star = s0 + v * T + (v * delta_v) / (2 * np.sqrt(a_max * b))
    acc = a_max * (1 - (v / v0) ** delta - (s_star / s) ** 2)
    # Mean and standard deviation
    mu = 0
    sigma = 0.5
    # Generate one sample
    value = np.random.normal(mu, sigma)
    return acc+value

# Simulation loop
dt = 0.1          # time step (s)
sim_time = 50     # total simulation time (s)
steps = int(sim_time / dt)

# Initial conditions for leader and follower
def generate_vehicle_trajectory(AtkFlag):
    # Simulation settings
    
    
    
    x_lead = np.zeros(steps)
    v_lead = np.ones(steps) * 25  # constant speed of leader (m/s)
    x_lead[0] = 100
    
    x_follow = np.zeros(steps)
    v_follow = np.zeros(steps)
    x_follow[0] = 0
    v_follow[0] = 0
    
    for t in range(steps - 1):
        s = x_lead[t] - x_follow[t]
        delta_v = v_follow[t] - v_lead[t]
        if AtkFlag==0:
            acc = idm_acceleration(s, v_follow[t], delta_v)
        else:
            acc = idm_acceleration_under_attack(s, v_follow[t], delta_v)
        
        
        v_follow[t+1] = v_follow[t] + acc * dt
        x_follow[t+1] = x_follow[t] + v_follow[t] * dt
        
        x_lead[t+1] = x_lead[t] + v_lead[t] * dt  # leader moves at constant speed
    return x_lead,x_follow, v_follow

x_lead,x_follow,v_follow=generate_vehicle_trajectory(AtkFlag=0)
x_lead_atk,x_follow_atk,v_follow_atk=generate_vehicle_trajectory(AtkFlag=1)

# Plotting
plt.figure(figsize=(10, 5))
plt.plot(np.arange(steps) * dt, x_lead, label='Leader')
plt.plot(np.arange(steps) * dt, x_follow, label='Follower', color='black')
plt.plot(np.arange(steps) * dt, x_follow_atk, label='Follower', color='red')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title('Car-Following Simulation (IDM)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

plt.figure(figsize=(10, 5))

plt.plot(np.arange(steps) * dt, v_follow, label='Follower', color='black')
plt.plot(np.arange(steps) * dt, v_follow_atk, label='Follower', color='red')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title('Car-Following Simulation (IDM)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
