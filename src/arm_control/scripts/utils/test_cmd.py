
from casadi import *

# 创建优化变量
x = MX.sym('x')
y = MX.sym('y')
vars = vertcat(x, y)

# 定义目标函数
f = (x - 1)**2 + (y - 2.5)**2

# 定义约束
g = [x - y, x + y - 3]

# 创建 NLP 问题
nlp = {'x': vars, 'f': f, 'g': vertcat(*g)}

# 创建求解器（使用 IPOPT）
solver = nlpsol('solver', 'ipopt', nlp)

# 设置初始猜测和约束边界
x0 = [0, 0]
lbg = [0, 0]
ubg = [0, 0]

# 求解
sol = solver(x0=x0, lbg=lbg, ubg=ubg)

# 输出结果
print("Optimal solution:")
print(sol['x'])
