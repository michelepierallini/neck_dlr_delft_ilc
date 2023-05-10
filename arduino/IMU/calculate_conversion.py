import numpy as np

# All units in mm
# See CAD file for true definition

r1 = 10
r2 = 5
l1 = 25
l2 = 0
l3 = 35.25 - l1

l4 = np.sqrt(l1**2 + l2**2)
alpha1 = np.arctan(l2/l1)
phi = np.arcsin((r1+r2)/l4)
alpha = phi - alpha1

l5 = np.sqrt(l3**2 + l2**2)
beta1 = np.arctan(l2/l3)
beta2 = np.arcsin(r2/l5)
beta = beta2 - beta1

conversion = np.sin(alpha) + np.sin(beta)

print("For 1N on the tendon, you get", conversion, "N on the loadcell")