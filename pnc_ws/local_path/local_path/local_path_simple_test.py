from local_opt_compiled import CompiledLocalOpt
from all_settings.all_settings import LocalOptSettings
import math
import numpy as np
import matplotlib.pyplot as plt

clo = CompiledLocalOpt(**LocalOptSettings)
clo.construct_solver(generate_c=False, compile_c=False, use_c=False, gcc_opt_flag='-O3')
left = np.ndarray(shape=(10,2), buffer=np.array([(6 * math.cos(i/10), 6 * math.sin(i/10)) for i in range(10)]))
right = np.ndarray(shape=(10,2), buffer=np.array([(10 * math.cos(i/10), 10 * math.sin(i/10)) for i in range(10)]))

curr_state = np.ndarray(shape=(1,4), buffer=np.array([8, 0, math.pi / 2, 2]))
print(left, right, curr_state)
plt.plot(*left.T)
plt.plot(*right.T)

result = clo.solve(left, right, curr_state)
states, controls = clo.to_constant_tgrid(0.05, **result)
print(states.shape)
print(result['t'])
scat = plt.scatter(*states[:, :2].T, c=states[:, 2])
# plt.colorbar(scat, )
print(clo.solve(left, right, curr_state))
plt.plot(*result['z'][:, :2].T)

plt.show()