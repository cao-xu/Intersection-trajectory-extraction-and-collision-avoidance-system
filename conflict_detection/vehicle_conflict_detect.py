import math
import time

import argparse
import os
import sys

import winsound

from conflict_detection.SAT import *
from conflict_detection.conflict import Conflict
from traj_ext.postprocess_track import trajutil
from traj_ext.postprocess_track.trajectory import Trajectory, TrajPoint
from traj_ext.utils import mathutil


def main(args_input):
    print("############################################################")
    print("Conflict detection")
    print("############################################################\n")

    # ##########################################################
    # # Parse Arguments
    # ##########################################################
    argparser = argparse.ArgumentParser(
        description='Conflict detection')
    argparser.add_argument(
        '-traj',
        default='',
        help='Path of the trajectories csv')
    argparser.add_argument(
        '-time',
        default='',
        help='Path of the time csv')
    argparser.add_argument(
        '-output_dir',
        type=str,
        default='',
        help='Path of the output')
    # argparser.add_argument(
    #     '-current_time_ms',
    #     type=int,
    #     default= 0,
    #     help='Current time ms')# 输入当前时刻，要处理的时刻

    args = argparser.parse_args(args_input)

    # 将参数输入，调用冲突检测函数
    # test_two_vehicle_conflict_detect(args)
    # 修改为调用 run_conflict_detection(args),上面的是测试函数
    run_conflict_detection(args)


def compute_mid_coordinate(length, traj_point, direction):

    if traj_point.vx == 0 and traj_point.vy == 0:
        t = 1
    else:
        t = length / (math.sqrt(traj_point.vx ** 2 + traj_point.vy ** 2) * 2)
    if direction == 'front' :
        mid_x = traj_point.x + t * traj_point.vx
        mid_y = traj_point.y + t * traj_point.vy
    else:
        mid_x = traj_point.x - t * traj_point.vx
        mid_y = traj_point.y - t * traj_point.vy
    point_mid = TrajPoint(0, mid_x, mid_y, \
                            traj_point.vx, traj_point.vy, 0)
    return point_mid

def get_vehicle_front_or_rear_two_point(x0, y0, r0, x1, y1, r1):
    # circle 1: (x0, y0), radius r0
    # circle 2: (x1, y1), radius r1

    d = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)

    # non intersecting
    if d > r0 + r1:
        return None
    # One circle within other
    if d < abs(r0 - r1):
        return None
    # coincident circles
    if d == 0 and r0 == r1:
        return None
    else:
        a = (r0 ** 2 - r1 ** 2 + d ** 2) / (2 * d)
        h = math.sqrt(r0 ** 2 - a ** 2)
        x2 = x0 + a * (x1 - x0) / d
        y2 = y0 + a * (y1 - y0) / d
        x3 = x2 + h * (y1 - y0) / d
        y3 = y2 - h * (x1 - x0) / d

        x4 = x2 - h * (y1 - y0) / d
        y4 = y2 + h * (x1 - x0) / d

        return (x3, y3, x4, y4)

def check_ray_intersection(traj_point_A, traj_point_B):
    d_x = traj_point_B.x - traj_point_A.x
    d_y = traj_point_B.y - traj_point_A.y
    det = traj_point_B.vx * traj_point_A.vy - traj_point_B.vy * traj_point_A.vx
    u = (d_y * traj_point_B.vx - d_x * traj_point_B.vy) / det
    v = (d_y * traj_point_A.vx - d_x * traj_point_A.vy) / det
    return u, v

def two_vehicle_lateral_conflict_detect(traj_point_A, traj_point_B, A_length, A_width, B_length, B_width):
    # 设置返回值
    conflict_point = TrajPoint(0, 0, 0, 0, 0, 0)
    TTC = 9999
    conflict_type = 'no_conflict'

    point_A_1 = TrajPoint(0, 0, 0, traj_point_A.vx, traj_point_A.vy, 0)
    point_A_2 = TrajPoint(0, 0, 0, traj_point_A.vx, traj_point_A.vy, 0)
    point_B_1 = TrajPoint(0, 0, 0, traj_point_B.vx, traj_point_B.vy, 0)
    point_B_2 = TrajPoint(0, 0, 0, traj_point_B.vx, traj_point_B.vy, 0)

    # 计算车头中点坐标
    point_A_mid = compute_mid_coordinate(A_length, traj_point_A, direction = 'front')
    point_B_mid = compute_mid_coordinate(B_length, traj_point_B, direction = 'front')
    # 通过计算两个圆的交点得出车头1点和2点
    intersections_A = get_vehicle_front_or_rear_two_point(point_A_mid.x, point_A_mid.y, A_width / 2, \
                                                          traj_point_A.x, traj_point_A.y,\
                                                          math.sqrt((A_length / 2) ** 2 + (A_width / 2) ** 2))
    intersections_B = get_vehicle_front_or_rear_two_point(point_B_mid.x, point_B_mid.y, B_width / 2, \
                                                          traj_point_B.x, traj_point_B.y,\
                                                          math.sqrt((B_length / 2) ** 2 + (B_width / 2) ** 2))
    point_A_1.x, point_A_1.y, point_A_2.x, point_A_2.y = intersections_A
    point_B_1.x, point_B_1.y, point_B_2.x, point_B_2.y = intersections_B

    # 计算 A车 到达四个冲突点的时间，判断实际潜在冲突点
    # 算4个，时间最小的那个是实际潜在冲突点
    # A = A_start + velocity * u
    # B = B_start + velocity * v
    u1, v1 = check_ray_intersection(point_A_1, point_B_1)
    u2, v2 = check_ray_intersection(point_A_1, point_B_2)
    u3, v3 = check_ray_intersection(point_A_2, point_B_1)
    u4, v4 = check_ray_intersection(point_A_2, point_B_2)

    t1, t2, t3, t4 = 9999, 9999, 9999, 9999
    if u1 >= 0 and v1 >= 0:
        p = TrajPoint(0, 0, 0, 0, 0, 0)
        p.x = point_A_1.x + point_A_1.vx * u1
        p.y = point_A_1.y + point_A_1.vy * u1
        distance = math.sqrt((p.x - point_A_1.x) ** 2 + (p.y - point_A_1.y) ** 2)
        A_vehicle_abs_velocity = math.sqrt(point_A_1.vx ** 2 + point_A_1.vy ** 2)
        t1 = distance / A_vehicle_abs_velocity
        # print('u1:',u1)
        print('A_1点到达潜在冲突点的时间：', t1)

    if u2 >= 0 and v2 >= 0:
        p = TrajPoint(0, 0, 0, 0, 0, 0)
        p.x = point_A_1.x + point_A_1.vx * u2
        p.y = point_A_1.y + point_A_1.vy * u2
        distance = math.sqrt((p.x - point_A_1.x) ** 2 + (p.y - point_A_1.y) ** 2)
        A_vehicle_abs_velocity = math.sqrt(point_A_1.vx ** 2 + point_A_1.vy ** 2)
        t2 = distance / A_vehicle_abs_velocity
        # print('u2:',u2)
        print('A_1点到达潜在冲突点的时间：', t2)

    if u3 >= 0 and v3 >= 0:
        p = TrajPoint(0, 0, 0, 0, 0, 0)
        p.x = point_A_2.x + point_A_2.vx * u3
        p.y = point_A_2.y + point_A_2.vy * u3
        distance = math.sqrt((p.x - point_A_2.x) ** 2 + (p.y - point_A_2.y) ** 2)
        A_vehicle_abs_velocity = math.sqrt(point_A_2.vx ** 2 + point_A_2.vy ** 2)
        t3 = distance / A_vehicle_abs_velocity
        # print('u3:',u3)
        print('A_2点到达潜在冲突点的时间：', t3)

    if u4 >= 0 and v4 >= 0:
        p = TrajPoint(0, 0, 0, 0, 0, 0)
        p.x = point_A_2.x + point_A_2.vx * u4
        p.y = point_A_2.y + point_A_2.vy * u4
        distance = math.sqrt((p.x - point_A_2.x) ** 2 + (p.y - point_A_2.y) ** 2)
        A_vehicle_abs_velocity = math.sqrt(point_A_2.vx ** 2 + point_A_2.vy ** 2)
        t4 = distance / A_vehicle_abs_velocity
        # print('u4:',u4)
        print('A_2点到达潜在冲突点的时间：', t4)

    # 若果四个点都不存在，则无潜在冲突点，结束
    if (u1 < 0 or v1 < 0) and (u2 < 0 or v2 < 0) and (u3 < 0 or v3 < 0) and (u4 < 0 or v4 < 0):
        print('四个交点都没找到，即无潜在冲突点\n')
        return conflict_point, TTC, conflict_type

    T = [t1, t2, t3, t4]
    min_t = min(T)
    index = T.index(min(T))
    print('最短运行时间：', min_t, '序号：', index + 1)

    # 判断最小t对应的p点,保存实际的潜在冲突点坐标 conflict_point（x， y）
    conflict_point = TrajPoint(0, 0, 0, 0, 0, 0)
    A_start_point = TrajPoint(0, 0, 0, 0, 0, 0)
    B_start_point = TrajPoint(0, 0, 0, 0, 0, 0)
    if index == 0:
        conflict_point.x = point_A_1.x + point_A_1.vx * u1
        conflict_point.y = point_A_1.y + point_A_1.vy * u1
        A_start_point = TrajPoint(0, point_A_1.x, point_A_1.y, point_A_1.vx, point_A_1.vy, 0)
        B_start_point = TrajPoint(0, point_B_1.x, point_B_1.y, point_B_1.vx, point_B_1.vy, 0)
    if index == 1:
        conflict_point.x = point_A_1.x + point_A_1.vx * u2
        conflict_point.y = point_A_1.y + point_A_1.vy * u2
        A_start_point = TrajPoint(0, point_A_1.x, point_A_1.y, point_A_1.vx, point_A_1.vy, 0)
        B_start_point = TrajPoint(0, point_B_2.x, point_B_2.y, point_B_2.vx, point_B_2.vy, 0)
    if index == 2:
        conflict_point.x = point_A_2.x + point_A_2.vx * u3
        conflict_point.y = point_A_2.y + point_A_2.vy * u3
        A_start_point = TrajPoint(0, point_A_2.x, point_A_2.y, point_A_2.vx, point_A_2.vy, 0)
        B_start_point = TrajPoint(0, point_B_1.x, point_B_1.y, point_B_1.vx, point_B_1.vy, 0)
    if index == 3:
        conflict_point.x = point_A_2.x + point_A_2.vx * u4
        conflict_point.y = point_A_2.y + point_A_2.vy * u4
        A_start_point = TrajPoint(0, point_A_2.x, point_A_2.y, point_A_2.vx, point_A_2.vy, 0)
        B_start_point = TrajPoint(0, point_B_2.x, point_B_2.y, point_B_2.vx, point_B_2.vy, 0)

    # 计算判断两辆车到达交点的时间片段是否发生重叠
    distance_A = math.sqrt((conflict_point.x - A_start_point.x) ** 2 + (conflict_point.y - A_start_point.y) ** 2)
    A_vehicle_velocity = math.sqrt(point_A_1.vx ** 2 + point_A_1.vy ** 2)
    t_As = distance_A / A_vehicle_velocity  # A车到达起始时间
    print('潜在冲突点坐标：x = {x}，y = {y}'.format(x=conflict_point.x, y=conflict_point.y))
    print('t_As:', t_As)
    distance_B = math.sqrt((conflict_point.x - B_start_point.x) ** 2 + (conflict_point.y - B_start_point.y) ** 2)
    B_vehicle_velocity = math.sqrt(point_B_1.vx ** 2 + point_B_1.vy ** 2)
    t_Bs = distance_B / B_vehicle_velocity  # B车到达起始时间
    print('t_Bs:', t_Bs)

    TTC = 9999
    if t_As - t_Bs < 0:  # A车先到的情况
        t_Ae = t_As + A_length / A_vehicle_velocity
        if t_Ae - t_Bs < 0:
            # A 车走完 B车才到，无冲突
            print('t_Ae:', t_Ae)
            print('A车先到并安全通过，无侧向冲突\n')
        else:
            # 有冲突，TTC为 B车还有多久到的时间
            TTC = t_Bs
            conflict_type = 'lateral_conflict'
            print('有侧向冲突,冲突点坐标为：x = {x}，y = {y}'.format(x=conflict_point.x, y=conflict_point.y), 'TTC为：', TTC)

    else:  # B车先到的情况
        t_Be = t_Bs + B_length / B_vehicle_velocity
        if t_Be - t_As < 0:
            print('t_Be:', t_Be)
            print('B车先到并安全通过，无侧向冲突\n')
        else:
            TTC = t_As
            conflict_type = 'lateral_conflict'
            print('有侧向冲突,冲突点坐标为：x = {x}，y = {y}'.format(x=conflict_point.x, y=conflict_point.y), 'TTC为：', TTC)

    # 若存在冲突，则输出 冲突点坐标，TTC
    return conflict_point, TTC, conflict_type

def is_point_left_of_ray(check_point_x, check_point_y, ray_s_x, ray_s_y, ray_e_x, ray_e_y):
    # 若返回的为 1 则 在左边，否则 在右边
    return (check_point_y - ray_s_y) * (ray_e_x - ray_s_x) > (check_point_x - ray_s_x) * (ray_e_y - ray_s_y)

def compute_A3_B12_rear_conflict_ttc(point_A_3, point_B_2, point_B_1):
    v_A = math.sqrt(point_A_3.vx ** 2 + point_A_3.vy ** 2)
    v_B = math.sqrt(point_B_2.vx ** 2 + point_B_2.vy ** 2)
    v_sum = v_A + v_B
    det_y_A3_B1 = point_A_3.y - point_B_1.y
    det_x_A3_B1 = point_A_3.x - point_B_1.x
    det_y_B2_A3 = point_B_2.y - point_A_3.y
    det_x_B2_A3 = point_B_2.x - point_A_3.x
    TTC = (det_x_A3_B1 * det_y_B2_A3 - det_y_A3_B1 * det_x_B2_A3) / \
          ((det_y_A3_B1 + det_x_B2_A3 - det_x_A3_B1 - det_y_B2_A3) * v_sum)
    return TTC

def compute_B2_A34_rear_conflict_ttc(point_B_2, point_A_3, point_A_4):
    v_A = math.sqrt(point_A_3.vx ** 2 + point_A_3.vy ** 2)
    v_B = math.sqrt(point_B_2.vx ** 2 + point_B_2.vy ** 2)
    v_sum = v_A + v_B
    det_y_B2_A3 = point_B_2.y - point_A_3.y
    det_x_B2_A3 = point_B_2.x - point_A_3.x
    det_y_A4_B2 = point_A_4.y - point_B_2.y
    det_x_A4_B2 = point_A_4.x - point_B_2.x
    TTC = (det_x_B2_A3 * det_y_A4_B2 - det_y_B2_A3 * det_x_A4_B2) / \
          ((det_y_B2_A3 + det_x_A4_B2 - det_x_B2_A3 - det_y_A4_B2) * v_sum)
    return TTC

def two_vehicle_rear_end_conflict_detect(traj_point_A, traj_point_B, A_length, A_width, B_length, B_width):
    # 设置返回值
    conflict_point = TrajPoint(0, 0, 0, 0, 0, 0)
    TTC = 9999
    conflict_type = 'no_conflict'

    # 判断两车的车尾3、4点是否存在交点（潜在冲突点）
        # 判断谁是先到的车，通过计算到达最远的A点谁先到，作为A车，在接下来的判断中取3、4点检测是否会发生追尾
    point_A_3 = TrajPoint(0, 0, 0, traj_point_A.vx, traj_point_A.vy, 0)
    point_A_4 = TrajPoint(0, 0, 0, traj_point_A.vx, traj_point_A.vy, 0)
    point_B_3 = TrajPoint(0, 0, 0, traj_point_B.vx, traj_point_B.vy, 0)
    point_B_4 = TrajPoint(0, 0, 0, traj_point_B.vx, traj_point_B.vy, 0)

    # 计算车头中点坐标
    point_A_mid = compute_mid_coordinate(A_length, traj_point_A, direction = 'rear')
    point_B_mid = compute_mid_coordinate(B_length, traj_point_B, direction = 'rear')
    # 通过计算两个圆的交点得出车尾3点和4点
    intersections_A = get_vehicle_front_or_rear_two_point(point_A_mid.x, point_A_mid.y, A_width / 2, \
                                                          traj_point_A.x, traj_point_A.y,\
                                                          math.sqrt((A_length / 2) ** 2 + (A_width / 2) ** 2))
    intersections_B = get_vehicle_front_or_rear_two_point(point_B_mid.x, point_B_mid.y, B_width / 2, \
                                                          traj_point_B.x, traj_point_B.y,\
                                                          math.sqrt((B_length / 2) ** 2 + (B_width / 2) ** 2))
    point_A_3.x, point_A_3.y, point_A_4.x, point_A_4.y = intersections_A
    point_B_3.x, point_B_3.y, point_B_4.x, point_B_4.y = intersections_B

    # 计算 A车 到达四个冲突点的时间，用于判断谁是先到的车（即存在追尾条件的A车）
    # 取 到达时间最大的
    u1, v1 = check_ray_intersection(point_A_3, point_B_3)
    u2, v2 = check_ray_intersection(point_A_3, point_B_4)
    u3, v3 = check_ray_intersection(point_A_4, point_B_3)
    u4, v4 = check_ray_intersection(point_A_4, point_B_4)

    t1, t2, t3, t4 = -1, -1, -1, -1

    if u1 >= 0 and v1 >= 0:
        t1 = u1
        print('A_3点到达潜在冲突点的时间：', t1)

    if u2 >= 0 and v2 >= 0:
        t2 = u2
        print('A_3点到达潜在冲突点的时间：', t2)

    if u3 >= 0 and v3 >= 0:
        t3 = u3
        print('A_4点到达潜在冲突点的时间：', t3)

    if u4 >= 0 and v4 >= 0:
        t4 = u4
        print('A_4点到达潜在冲突点的时间：', t4)

    # 若果四个点都不存在，则无潜在冲突点，结束
    if (u1 < 0 or v1 < 0) and (u2 < 0 or v2 < 0) and (u3 < 0 or v3 < 0) and (u4 < 0 or v4 < 0):
        print('四个交点都没找到，即无追尾冲突点\n')
        return conflict_point, TTC, conflict_type

    T = [t1, t2, t3, t4]
    max_t = max(T)
    index = T.index(max(T))
    print('最长运行时间：', max_t, '序号：', index + 1)

    # 最大的t对应 P1点 ，计算A、B分别通过 P1的时间，谁的小，谁是A车
    # 定义 vehicle_A 为先到车， vehicle_B 为后到车
    # 下面的程序判断谁是先到车，并把相应数据赋值给对应的车
    vehicle_A = TrajPoint(0, 0, 0, 0, 0, 0)
    vehicle_B = TrajPoint(0, 0, 0, 0, 0, 0)
    vehicle_A_length = 4
    vehicle_B_length = 4
    vehicle_A_width = 1.8
    vehicle_B_width = 1.8
    if index == 0:
        # u,v 分别是A车、B车到达 P1 点的时间
        if u1 - v1 < 0 :
            vehicle_A = traj_point_A
            vehicle_B = traj_point_B
            vehicle_A_length = A_length
            vehicle_B_length = B_length
            vehicle_A_width = A_width
            vehicle_B_width = B_width
        else:
            # B点 对应的数据才是 先到车 的数据
            vehicle_A = traj_point_B
            vehicle_B = traj_point_A
            vehicle_A_length = B_length
            vehicle_B_length = A_length
            vehicle_A_width = B_width
            vehicle_B_width = A_width
    if index == 1:
        # u,v 分别是A车、B车到达 P1 点的时间
        if u2 - v2 < 0:
            vehicle_A = traj_point_A
            vehicle_B = traj_point_B
            vehicle_A_length = A_length
            vehicle_B_length = B_length
            vehicle_A_width = A_width
            vehicle_B_width = B_width
        else:
            # B点 对应的数据才是 先到车 的数据
            vehicle_A = traj_point_B
            vehicle_B = traj_point_A
            vehicle_A_length = B_length
            vehicle_B_length = A_length
            vehicle_A_width = B_width
            vehicle_B_width = A_width
    if index == 2:
        # u,v 分别是A车、B车到达 P1 点的时间
        if u3 - v3 < 0:
            vehicle_A = traj_point_A
            vehicle_B = traj_point_B
            vehicle_A_length = A_length
            vehicle_B_length = B_length
            vehicle_A_width = A_width
            vehicle_B_width = B_width
        else:
            # B点 对应的数据才是 先到车 的数据
            vehicle_A = traj_point_B
            vehicle_B = traj_point_A
            vehicle_A_length = B_length
            vehicle_B_length = A_length
            vehicle_A_width = B_width
            vehicle_B_width = A_width
    if index == 3:
        # u,v 分别是A车、B车到达 P1 点的时间
        if u4 - v4 < 0:
            vehicle_A = traj_point_A
            vehicle_B = traj_point_B
            vehicle_A_length = A_length
            vehicle_B_length = B_length
            vehicle_A_width = A_width
            vehicle_B_width = B_width
        else:
            # B点 对应的数据才是 先到车 的数据
            vehicle_A = traj_point_B
            vehicle_B = traj_point_A
            vehicle_A_length = B_length
            vehicle_B_length = A_length
            vehicle_A_width = B_width
            vehicle_B_width = A_width


    # 计算vehicleA 的3、4点与vehicleB 的1、2点的交点，判断是否存在交点，若存在，
    # 首先，分辨1、2点与3、4点，
    # 判断是否存在P2点
    # 通过计算确定P2点、P1点，计算通过P2点的时间比较是否会发生第一阶段的追尾
    # 判断是否存在P1点
    # 其次，计算通过P1点的时间比较是否会发生第二阶段的追尾

    # 在A车向右开的情况下

    # 添加 分辨 1、2点与3、4点的方法
    point_A_3 = TrajPoint(0, 0, 0, vehicle_A.vx, vehicle_A.vy, 0)
    point_A_4 = TrajPoint(0, 0, 0, vehicle_A.vx, vehicle_A.vy, 0)
    point_B_1 = TrajPoint(0, 0, 0, vehicle_B.vx, vehicle_B.vy, 0)
    point_B_2 = TrajPoint(0, 0, 0, vehicle_B.vx, vehicle_B.vy, 0)

    # 计算 A车车尾、B车车头 中点坐标
    point_A_mid = compute_mid_coordinate(vehicle_A_length, vehicle_A, direction='rear')
    point_B_mid = compute_mid_coordinate(vehicle_B_length, vehicle_B, direction='front')
    # 通过计算两个圆的交点得出车尾3点和4点
    intersections_A = get_vehicle_front_or_rear_two_point(point_A_mid.x, point_A_mid.y, vehicle_A_width / 2, \
                                                          vehicle_A.x, vehicle_A.y, \
                                                          math.sqrt((vehicle_A_length / 2) ** 2 + (vehicle_A_width / 2) ** 2))
    # 计算车头 1、2点
    intersections_B = get_vehicle_front_or_rear_two_point(point_B_mid.x, point_B_mid.y, vehicle_B_width / 2, \
                                                          vehicle_B.x, vehicle_B.y, \
                                                          math.sqrt((vehicle_B_length / 2) ** 2 + (vehicle_B_width / 2) ** 2))
    point_A_3.x, point_A_3.y, point_A_4.x, point_A_4.y = intersections_A
    point_B_1.x, point_B_1.y, point_B_2.x, point_B_2.y = intersections_B

    # 首先，分辨B车1、2点与A车3、4点，
    # 判断是否存在P2点

    # 判断B1 与 B2点
    B_ray_end_point_x = vehicle_B.x + vehicle_B.vx * 1
    B_ray_end_point_y = vehicle_B.y + vehicle_B.vy * 1
    if not is_point_left_of_ray(point_B_1.x, point_B_1.y, vehicle_B.x, vehicle_B.y, \
                                B_ray_end_point_x, B_ray_end_point_y):
        # 当前B1点 在射线右侧，需要交换数据
        tmp_x = point_B_1.x
        tmp_y = point_B_1.y
        point_B_1.x = point_B_2.x
        point_B_1.y = point_B_2.y
        point_B_2.x = tmp_x
        point_B_2.y = tmp_y
    # 判断 A3 与 A4
    A_ray_end_point_x = vehicle_A.x + vehicle_A.vx * 1
    A_ray_end_point_y = vehicle_A.y + vehicle_A.vy * 1
    if not is_point_left_of_ray(point_A_4.x, point_A_4.y, vehicle_A.x, vehicle_A.y, \
                                A_ray_end_point_x, A_ray_end_point_y):
        # 当前 A4 不在A车左侧，需要交换数据
        tmp_x = point_A_4.x
        tmp_y = point_A_4.y
        point_A_4.x = point_A_3.x
        point_A_4.y = point_A_3.y
        point_A_3.x = tmp_x
        point_A_3.y = tmp_y

    # 由B2 、 A3 组成的射线判断是否存在P2点，存在则计算t_A3_P2 - t_B2_P2 < 0
    u_a3, v_b2 = check_ray_intersection(point_A_3, point_B_2)
    # A4 与 B2 存在交点
    if u_a3 >= 0 and v_b2 >= 0:
        t_A3_P2 = u_a3
        t_B2_P2 = v_b2

        #存在第一种追尾冲突
        if not t_A3_P2 - t_B2_P2 < 0:
            # 不满足条件，则存在第一阶段的追尾，计算 TTC、冲突点坐标，修改冲突类型conflict_type
            TTC = compute_A3_B12_rear_conflict_ttc(point_A_3, point_B_2, point_B_1)
            conflict_point.x = point_A_3.x + point_A_3.vx * TTC
            conflict_point.y = point_A_3.y + point_A_3.vy * TTC
            conflict_type = 'A3_B12_rear_conflict'
            print('检测到第一类A3_B12的追尾冲突， TTC:',TTC,'s',\
                  '冲突点坐标：x = {x}，y = {y}\n'.format(x=conflict_point.x, y=conflict_point.y))
            return conflict_point, TTC, conflict_type

    # 若不存在第一种追尾，继续判断下一种追尾
    # 由B2、A4组成的P1点是否存在，存在再判断是否存在追尾冲突
    u_a4, v_b2 = check_ray_intersection(point_A_4, point_B_2)
    # A4 与 B2 存在 P1 交点
    if u_a4 >= 0 and v_b2 >= 0:
        t_A4_P1 = u_a4
        t_B2_P1 = v_b2

         # 存在第二种追尾冲突
        if not t_A4_P1 - t_B2_P1 < 0:
            TTC = compute_B2_A34_rear_conflict_ttc(point_B_2, point_A_3, point_A_4)
            conflict_point.x = point_B_2.x + point_B_2.vx * TTC
            conflict_point.y = point_B_2.y + point_B_2.vy * TTC
            conflict_type = 'B2_A34_rear_conflict'
            print('检测到第二类B2_A34的追尾冲突， TTC:', TTC, 's', \
                  '冲突点坐标：x = {x}，y = {y}\n'.format(x=conflict_point.x, y=conflict_point.y))
            return conflict_point, TTC, conflict_type

    # 不存在第二种冲突，也不存在第一种追尾冲突，返回默认三参数
    print('未检测到追尾冲突！\n')
    return conflict_point, TTC, conflict_type

def two_vehicle_conflict_detect(traj_i, traj_j, time_ms):
    '''
    实现两车冲突检测，若存在冲突则返回冲突点与TTC
    :param traj_i: 轨迹1
    :param traj_j: 轨迹2
    :param time_ms: 当前时刻
    :return: TrajPoint类的冲突点，TTC，冲突类型（侧向冲突/追尾冲突）
    '''

    # traj_point_A = TrajPoint(0,0,0,0,0,0) # 初始化时以“0”为轨迹点参数
    # traj_point_B = TrajPoint(0,0,0,0,0,0) # 初始化时以“0”为轨迹点参数
    # A_length, A_width, A_height = 0, 0, 0
    # B_length, B_width, B_height = 0, 0, 0

    # 获取两车当前帧的轨迹点数据
    # 获得的两个轨迹点属性(time_ms, x, y, vx, vy, psi_rad)
    traj_point_A = traj_i.get_point_at_timestamp(time_ms)  # 某一时刻
    A_length, A_width, A_height = traj_i.get_size()
    # print(traj_point_A.x, traj_point_A.y, traj_point_A.vx, traj_point_A.vy)
    traj_point_B = traj_j.get_point_at_timestamp(time_ms)
    B_length, B_width, B_height = traj_j.get_size()
    # print(traj_point_B.x, traj_point_B.y, traj_point_B.vx, traj_point_B.vy)

    # 返回参数初始化
    conflict_point = TrajPoint(0, 0, 0, 0, 0, 0)
    TTC = 9999
    conflict_type = 'no_conflict'

    # 判断是否当前time_ms是否存在轨迹点，若存在，再进行下面的判断
    if (traj_point_A is None) or (traj_point_B is None) :

        return conflict_point, TTC, conflict_type

    else:
        # 首先,判断侧向冲突
        conflict_point, TTC, conflict_type = two_vehicle_lateral_conflict_detect(traj_point_A, traj_point_B, \
                                                                                 A_length, A_width, B_length, B_width)
        # 若存在冲突，则返回（冲突点、TTC、冲突类型）
        if TTC != 9999 and conflict_point.x != 0 and conflict_point.y != 0:

            return conflict_point, TTC, conflict_type

        # 其次,判断追尾冲突
        conflict_point, TTC, conflict_type = two_vehicle_rear_end_conflict_detect(traj_point_A, traj_point_B, \
                                                                                  A_length, A_width, B_length, B_width)

        # 若存在，则返回（冲突点、T型TC、冲突类）
        if TTC != 9999 and conflict_point.x != 0 and conflict_point.y != 0:

            return conflict_point, TTC, conflict_type

        # 若不存在，则返回默认的（冲突点、TTC、冲突类型），在外面的接收函数，通过判断字符内容，辨别是否存在冲突
        else:

            return conflict_point, TTC, conflict_type

def vehicle_bounding_point(point_x, point_y, length, width, vx, vy):

    # 计算4个角点的坐标，返回轨迹点TrajPoint类的对象

    # 计算中点坐标，带入计算车头、车尾的两点坐标
    center_point = TrajPoint(0, point_x, point_y, vx, vy, 0)

    front_mid_point = compute_mid_coordinate(length, center_point, 'front')
    rear_mid_point = compute_mid_coordinate(length, center_point, 'rear')

    point_1 = TrajPoint(0, 0, 0, vx, vy, 0)
    point_2 = TrajPoint(0, 0, 0, vx, vy, 0)
    point_3 = TrajPoint(0, 0, 0, vx, vy, 0)
    point_4 = TrajPoint(0, 0, 0, vx, vy, 0)

    point_1.x, point_1.y, point_2.x, point_2.y = get_vehicle_front_or_rear_two_point(front_mid_point.x,\
                                                                                     front_mid_point.y,\
                                                                                     width / 2, \
                                                                                     center_point.x, \
                                                                                     center_point.y, \
                                                                                     math.sqrt((length / 2) ** 2 + (width / 2) ** 2))
    point_3.x, point_3.y, point_4.x, point_4.y = get_vehicle_front_or_rear_two_point(rear_mid_point.x,\
                                                                                     rear_mid_point.y, \
                                                                                     width / 2, \
                                                                                     center_point.x, \
                                                                                     center_point.y, \
                                                                                     math.sqrt((length / 2) ** 2 + (width / 2) ** 2))
    # 判断在速度方向左边的分别是 2点 与 3点，右边的是 1点、4点
    ray_end_point_x = center_point.x + center_point.vx * 1
    ray_end_point_y = center_point.y + center_point.vy * 1
    if not is_point_left_of_ray(point_2.x, point_2.y, center_point.x, center_point.y, \
                                ray_end_point_x, ray_end_point_y):
        tmp_x = point_2.x
        tmp_y = point_2.y
        point_2.x = point_1.x
        point_2.y = point_1.y
        point_1.x = tmp_x
        point_1.y = tmp_y

    if not is_point_left_of_ray(point_3.x, point_3.y, center_point.x, center_point.y, \
                                ray_end_point_x, ray_end_point_y):
        tmp_x = point_3.x
        tmp_y = point_3.y
        point_3.x = point_4.x
        point_3.y = point_4.y
        point_4.x = tmp_x
        point_4.y = tmp_y

    # 返回 1、2、3、4四个TrajPoint类的对象
    return point_1, point_2, point_3, point_4

# 2020.12.05 基于分轴定理的碰撞检测（预测）
def SAT_based_two_vehicle_conflicts_detect(predict_time_length, time_step, traj_i, traj_j, time_ms):
    '''

    :param predict_time_length:
    :param time_step:
    :param traj_i:
    :param traj_j:
    :param time_ms:
    :return: TTC、A_point, B_point
    '''

    # 获取两车当前帧的轨迹点数据
    # 获得的两个轨迹点属性(time_ms, x, y, vx, vy, psi_rad)
    traj_point_A = traj_i.get_point_at_timestamp(time_ms)  # 某一时刻
    A_length, A_width, A_height = traj_i.get_size()
    # print(traj_point_A.x, traj_point_A.y, traj_point_A.vx, traj_point_A.vy)
    traj_point_B = traj_j.get_point_at_timestamp(time_ms)
    B_length, B_width, B_height = traj_j.get_size()
    # print(traj_point_B.x, traj_point_B.y, traj_point_B.vx, traj_point_B.vy)

    # 设置返回值，发生碰撞时的A、B点
    conflict_point_A = TrajPoint(0, 0, 0, traj_point_A.vx, traj_point_A.vy, 0)
    conflict_point_B = TrajPoint(0, 0, 0, traj_point_B.vx, traj_point_B.vy, 0)
    TTC = 9999

    if traj_point_A.vx is None or traj_point_A.vy is None or traj_point_B.vx is None or traj_point_B.vy is None\
            or traj_point_A.vx == 0 or traj_point_A.vy == 0 or traj_point_B.vx == 0 or traj_point_B.vy == 0:

        return TTC, conflict_point_A, conflict_point_B

    # 对于每个时间步长，计算A、B两车的位置坐标点
    predict_time = time_step
    while predict_time <= predict_time_length:

        # 获取四个角点坐标，逆时针设置A、B车的边界点
        point_A_x = traj_point_A.x + traj_point_A.vx * predict_time
        point_A_y = traj_point_A.y + traj_point_A.vy * predict_time
        point_B_x = traj_point_B.x + traj_point_B.vx * predict_time
        point_B_y = traj_point_B.y + traj_point_B.vy * predict_time
        # 1、2、3、4是逆时针设置的四个轨迹点
        point_A_1, point_A_2, point_A_3, point_A_4 = vehicle_bounding_point(point_A_x, point_A_y, \
                                                                            A_length, A_width, \
                                                                            traj_point_A.vx, traj_point_A.vy)
        point_B_1, point_B_2, point_B_3, point_B_4 = vehicle_bounding_point(point_B_x, point_B_y,\
                                                                            B_length, B_width, \
                                                                            traj_point_B.vx, traj_point_B.vy)
        rectangle_A = [(point_A_1.x, point_A_1.y), (point_A_2.x, point_A_2.y), \
                       (point_A_3.x, point_A_3.y), (point_A_4.x, point_A_4.y)]
        rectangle_B = [(point_B_1.x, point_B_1.y), (point_B_2.x, point_B_2.y), \
                       (point_B_3.x, point_B_3.y), (point_B_4.x, point_B_4.y)]

        # 判断是否发生碰撞，若发生，则保存时间点
        if separating_axis_theorem(rectangle_A, rectangle_B):

            TTC = predict_time
            # 根据发生碰撞的时间，计算A、B两车的中心坐标点，若在整个时间长度内不存在冲突，则返回函数开头默认的冲突点，即设置坐标为A（0,0）B（0,0）
            conflict_point_A.x = point_A_x
            conflict_point_A.y = point_A_y
            conflict_point_B.x = point_B_x
            conflict_point_B.y = point_B_y

            return TTC, conflict_point_A, conflict_point_B

        predict_time += time_step

    # 若在整个时段内未检测到冲突，则返回默认值
    return TTC, conflict_point_A, conflict_point_B

def run_conflict_detection(config):

    # Create output folder
    output_dir = config.output_dir
    output_dir = os.path.join(output_dir, 'detected_conflict')
    os.makedirs(output_dir, exist_ok=True)

    # 读取轨迹
    traj_list = Trajectory.read_trajectory_panda_csv(config.traj)

    # Time,读取时刻数组
    list_times_ms_path = config.time
    if list_times_ms_path == '':
        list_times_ms_path = config.traj.replace('traj.csv', 'time_traj.csv')
    if not os.path.isfile(list_times_ms_path):
        print('[ERROR]: Traj time file not found: {}'.format(list_times_ms_path))
        return
    list_times_ms = trajutil.read_time_list_csv(list_times_ms_path)

    # 参数初始化,从0时刻至视频结束均检测冲突
    frame_index = 0
    # 所有时刻所有的冲突
    conflict_list = []

    # 读取每一帧画面的轨迹，进行冲突检测
    while True:

        #######################################################
        ## detect conflict
        #######################################################

        # Get Time ms
        frame_index = mathutil.clip(frame_index, 0, len(list_times_ms) - 1)  # 剪辑返回的是帧序，保证帧序在0到最大之间
        time_ms = list_times_ms[frame_index]  # 读取当前的时刻，用于遍历所有存在轨迹的时间段
        print("正在检测第{}ms的画面...\n".format(time_ms))

        # 两两组合下所有两条轨迹组合遍历检测冲突
        for i, traj_i in enumerate(traj_list):
            # i 从 0 到 n-2 即 遍历 轨迹1 至轨迹 n - 1
            if i >= 0 and i <= len(traj_list) - 2:
                # 判断当前时刻 轨迹 i 若存在轨迹点，则继续
                if not (traj_i.get_point_at_timestamp(time_ms) is None):

                    for j, traj_j in enumerate(traj_list):

                        if j >= i + 1 and j <= len(traj_list) - 1:
                            # 判断当前时刻 轨迹 j 若存在轨迹点， 则继续
                            if not (traj_j.get_point_at_timestamp(time_ms) is None):

                                # conflict_point, TTC, conflict_type = two_vehicle_conflict_detect(traj_i, traj_j, time_ms)

                                # 修改冲突检测的函数，基于分离轴定理预测冲突（侧向、追尾、车头与车头）
                                TTC, A_point, B_point = SAT_based_two_vehicle_conflicts_detect(5.0, \
                                                                                               0.01, \
                                                                                               traj_i, \
                                                                                               traj_j, \
                                                                                               time_ms)

                                # 若存在冲突（TTC 不为 9999）, 保存当前冲突到 list
                                if TTC != 9999 and A_point.x != 0 and A_point.y != 0 and B_point.x != 0 and B_point.y != 0:
                                    # 把当前的冲突数据 写入 Conflict 类

                                    # Conflict类重新设计成员变量
                                    tmp_data = Conflict(time_ms, A_point.x, A_point.y, B_point.x, B_point.y, \
                                                           TTC, traj_i.get_id(), traj_j.get_id())
                                    # 将冲突写入数组
                                    conflict_list.append(tmp_data)
                                    # 滴滴发出声音，报警有冲突
                                    winsound.Beep(500, 500)  # 第一个参数是音调 ，第二个参数是持续时间
                                    time.sleep(0.5)
                                    winsound.Beep(500, 500)
                                    print(time_ms,'ms时，轨迹',traj_i.get_id(),'与轨迹',traj_j.get_id(),\
                                          '存在冲突！TTC为{}s！\n\n'.format(TTC))


        # 完成所有时刻检测
        if frame_index == (len(list_times_ms)-1):
            name_prefix = 'conflict_detect'
            # 完成所有时刻冲突检测后保存冲突list到csv
            Conflict.write_conflict_panda_csv(output_dir, name_prefix, conflict_list)
            print('冲突检测完成！请查看文件')
            break
        # 当前时刻的冲突检测完成， 进行下一个时刻的检测
        else:
            frame_index += 1




if __name__ == '__main__':

    try:
        main(sys.argv[1:])
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')