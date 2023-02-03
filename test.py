import automate
import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv('2023-02-03_num_2_seed_45_5_vars/ro_0.216_ri_0.185_w_0.124_sw_0.025_n_4_E_10000000000.0_l_10000.0_rot_0.0_nodes.csv')
data_ex = data[data.nodetype==1]
data_in = data[data.nodetype==0]
x, y, z = data_ex.x, data_ex.y, data_ex.z
x1, y1, z1 = data_in.x, data_in.y, data_in.z
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(projection='3d')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
sca = ax.scatter(x, y, z)
sca = ax.scatter(x1, y1, z1)
plt.show()

# automate.run_model(r_out=0.3, r_in=0.2, width=0.1, spoke_width=0.04, num_spokes=4,
#                    init_angle=0, E=1e8, load=10000, meshsize=0.02, z_density=2,vis=True)