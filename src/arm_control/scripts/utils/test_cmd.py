#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import casadi as ca

# 创建一个矩阵
matrix = ca.SX([[1.0, 2.0], [3.0, 4.0]])

# 对矩阵中的每个元素进行指数运算
exp_matrix = ca.exp(matrix)

print("原矩阵:")
print(matrix)
print("指数矩阵:")
print(exp_matrix)