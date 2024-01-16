#!/usr/bin/env python
# encoding:utf8
import math 
import sympy as sp
body_tall = 2
target_plat_height = 0.03

# e0 = 0.13  # dabi length
# e1 = 0.23  # xaiobi length
# e2 = 0.33  # gripper length
# es = 0.10  # gripper jian to catch dian length

# # class Target:
# #     def __init__(self, x, y, z, phi=None, length=None, height=None):
# #         self.x = x
# #         self.y = y
# #         self.z = z - body_tall + target_plat_height
            
# #         self.phi = math.atan(self.x/self.y)
# #         self.height = self.z
# #         self.length = math.sqrt(self.x**2 + self.y**2)

# alpha, beta = sp.symbols('alpha beta')
# eq1 = e1 * sp.cos(beta) - e0 * sp.cos(alpha) - target_plat_height
# eq2 = e0 * sp.sin(alpha) + e1 * sp.sin(beta) + e2 - es - 0.5
# solutions = sp.solve((eq1,eq2), (alpha, beta))

# # a = round(solutions[0][0].evalf(),3)
# # b = round(solutions[0][1].evalf(),3)
# print(solutions[0][0])
# # print(b)

# # solution = sp.solve((eq1, eq2), (alpha, beta), dict=True)
# # for sol in solution:

# # if __name__ == "__main__":
# #     target = Target(10, 50, 100)
# #     alpha, beta = sp.symbols('alpha beta')
# #     eq1 = e1 * sp.cos(beta) - e0 * sp.cos(alpha) - target_plat_height
# #     eq2 = e0 * sp.sin(alpha) + e1 * sp.sin(beta) + e2 - es - target.length

# #     # solution = sp.solve((eq1, eq2), (alpha, beta), dict=True)
# #     # for sol in solution:

# #     #     print({sol[alpha]})
# #     #     print(solution[beta])

import math 
PI = 3.1415926535
e0 = 0.300  # dabi length
e1 = 0.355  # xaiobi length
e2 = 0.194  # gripper length
es = 0.050  # gripper jian to catch dian length
offset_length = 0.115 # arm_joint_2 to plat_gan
offset_high = 0.652   # map to plat_origin_height
body_tall = 1.600      # camera to map\
length = 0.5
# ARM_LENGTH_MAX = 0.913
# ARM_LENGTH_MIN = 0.35
ARM_LENGTH_MAX = e0 + e1 + e2 + offset_length - es  # 能够伸到的最远距离
ARM_LENGTH_MIN = e0 * math.sin(PI/4) +offset_length + e2 - es # 蜷缩距离
print(ARM_LENGTH_MAX)
print(ARM_LENGTH_MIN)
a, b = sp.symbols('alpha beta')
eq1 = e1 * sp.cos(b) - e0 * sp.cos(a) - target_plat_height
eq2 = e0 * sp.sin(a) + e1 * sp.sin(b) + e2 - es - length
solution = sp.solve((eq1, eq2), (a, b))

for a_val, b_val in solution:
    alpha = round(sp.re(a_val.evalf()), 3)
    beta = round(sp.re(b_val.evalf()), 3)
    if alpha > PI or beta > PI or alpha<0 or beta<0:
        pass
    else:
        break

print(alpha)
print(beta)