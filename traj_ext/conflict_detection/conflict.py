import cv2
import numpy as np
import os
import collections
import pandas as pd

from traj_ext.postprocess_track.trajectory import TrajPoint
from traj_ext.utils import cfgutil


# class Conflict_list():
#     def __init__(self):
#         self._conflict_list = []
#
#     def add_conflict_point(self, time_ms, x, y, TTC, track_id_A, track_id_B):
#
#         data = Conflict(time_ms, x, y, TTC, track_id_A, track_id_B)
#
#         self._conflict_list.append(data)
#
#     def get_list(self):
#
#         return self._conflict_list


class Conflict(object):
    # 静态变量
    id = 0

    def __init__(self, time_ms, x, y, TTC, track_id_A, track_id_B, conflict_type):
        Conflict.id += 1
        self._id = Conflict.id
        self._time_ms = time_ms
        self._x = x
        self._y = y
        self._TTC = TTC
        self._track_id_A = track_id_A
        self._track_id_B = track_id_B
        self._conflict_type = conflict_type
        self._color = (int(np.random.randint(0, 255, 1)[0]), int(np.random.randint(0, 255, 1)[0]), int(np.random.randint(0, 255, 1)[0]))
        # self._per_ms_conflict_list = per_ms_conflict_list

    # to do：
    # 模仿Trajectory类写 “写入csv文件的函数” 用于保存冲突使用
    @classmethod
    def write_conflict_panda_csv(cls, folder_path, name_prefix, conflict_list, list_times_ms):
        """Write conflicts into a single csv file

        Args:
            folder_path (TYPE): Description
            name_prefix (TYPE): Description
            conflict_list (TYPE): Description
            list_times_ms (TYPE): Description

        Returns:
            TYPE: Description

        Raises:
            NameError: Description
        """

        # Write conflicts
        csv_name = name_prefix + '.csv'
        df_conflict_path = os.path.join(folder_path, csv_name)

        dict_conflict_pd = collections.OrderedDict.fromkeys(['conflict_id', \
                                                         'timestamp_ms', \
                                                         'x', \
                                                         'y', \
                                                         'TTC', \
                                                         'track_id_A', \
                                                         'track_id_B', \
                                                         'conflict_type'])

        dict_conflict_pd['conflict_id'] = []
        dict_conflict_pd['timestamp_ms'] = []
        dict_conflict_pd['x'] = []
        dict_conflict_pd['y'] = []
        dict_conflict_pd['TTC'] = []
        dict_conflict_pd['track_id_A'] = []
        dict_conflict_pd['track_id_B'] = []
        dict_conflict_pd['conflict_type'] = []

        for conflict in conflict_list:

            # Define name:
            dict_conflict_pd['conflict_id'].append(conflict._id)
            dict_conflict_pd['timestamp_ms'].append(conflict._time_ms)
            dict_conflict_pd['x'].append(conflict._x)
            dict_conflict_pd['y'].append(conflict._y)
            dict_conflict_pd['TTC'].append(conflict._TTC)
            dict_conflict_pd['track_id_A'].append(conflict._track_id_A)
            dict_conflict_pd['track_id_B'].append(conflict._track_id_B)
            dict_conflict_pd['conflict_type'].append(conflict._conflict_type)

        # Create dataframe
        df_conflict = pd.DataFrame(dict_conflict_pd)

        # Sort by track_id
        df_conflict.sort_values(by = ['conflict_id'], inplace = True)

        # Write dataframe in csv
        df_conflict.to_csv(df_conflict_path, index= False)

        return


    # 模仿Trajectory类写 “读取csv文件的函数” 用于画图时使用
    @classmethod
    def read_conflict_panda_csv(cls, conflict_panda_csv_path):

        # Read dataframe with panda
        df = pd.read_csv(conflict_panda_csv_path)

        grouped = df.groupby(['timestamp_ms'], sort=False)

        # 设置数组的元素个数
        conflict_list = [None] * grouped.ngroups

        current_frame = 0

        for timestamp_ms, rows in grouped:

            # conflict_id = rows['conflict_id'].values[0]

            per_ms_conflict_list = []

            for index, row in rows.iterrows():

                conflict_id = row['conflict_id']
                time_ms = row['timestamp_ms']
                x = row['x']
                y = row['y']
                TTC = row['TTC']
                track_id_A = row['track_id_A']
                track_id_B = row['track_id_B']
                conflict_type = row['conflict_type']

                conflict_point = Conflict(time_ms, x, y, TTC, track_id_A, track_id_B, conflict_type)
                per_ms_conflict_list.append(conflict_point)
                # per_ms_conflict_list.add_conflict_point(time_ms, x, y, TTC, track_id_A, track_id_B)

            conflict_list[current_frame] = per_ms_conflict_list

            # Display status
            status_str = 'Reading conflict: {}/{}'.format(current_frame, grouped.ngroups)
            cfgutil.progress_bar(current_frame, grouped.ngroups, status_str)

            current_frame += 1

        return conflict_list

    def get_time_ms(self):

        return self._time_ms

    def get_x(self):

        return self._x

    def get_y(self):

        return self._y

    def get_TTC(self):

        return self._TTC

    def get_track_id_A(self):

        return self._track_id_A

    def get_track_id_B(self):

        return self._track_id_B

    def get_color(self):
        """Return the conflict color

        Returns:
            TYPE: tulpe
        """

        return self._color
    def get_conflict_type(self):

        return self._conflict_type

    # 模仿Trajectory的display_on_image_02time_ms（）写一个 画冲突点的函数
    def display_conflict_point_on_image(self, time_ms, cam_model, image_current_conflict, traj_list, TTC_label = True, conflict_type_label = True):

        # 判断图像存在 并且 TTC不为负值，才继续进行下面的画图
        if not (image_current_conflict is None) and (self._TTC > 0):

            # 依次画 冲突点 、 写TTC 、 画连线
            # 画冲突点
            pt_conflict = cam_model.project_points(np.array([(self._x, self._y, 0.0)])) # 将 平面坐标 转换为 原始镜头坐标
            pt_conflict = (int(pt_conflict[0]), int(pt_conflict[1]))
            # 冲突点 标为 红色 大实心圆
            image_current_conflict = cv2.circle(image_current_conflict, pt_conflict, 8, (0,0,255), -1)

            # 两辆车与 冲突点的连线
            # 获取两条轨迹，并获取当前时刻的轨迹点
            traj_point_A = TrajPoint(0, 0, 0, 0, 0, 0)
            traj_point_B = TrajPoint(0, 0, 0, 0, 0, 0)
            for traj in traj_list:

                if traj.get_id() == self._track_id_A:
                    traj_point_A = traj.get_point_at_timestamp(time_ms)

                if traj.get_id() == self._track_id_B:
                    traj_point_B = traj.get_point_at_timestamp(time_ms)

            pt_pix_A = cam_model.project_points(np.array([(traj_point_A.x, traj_point_A.y, 0.0)]))
            pt_pix_A = (int(pt_pix_A[0]), int(pt_pix_A[1]))
            pt_pix_B = cam_model.project_points(np.array([(traj_point_B.x, traj_point_B.y, 0.0)]))
            pt_pix_B = (int(pt_pix_B[0]), int(pt_pix_B[1]))
            # 画的是 红色 宽度为2像素的 实线
            image_current_conflict = cv2.line(image_current_conflict, pt_pix_A, pt_conflict, (0, 0, 255), 2)
            image_current_conflict = cv2.line(image_current_conflict, pt_pix_B, pt_conflict, (0, 0, 255), 2)
            # 两辆相关车辆用 蓝色 的 实线 相连
            image_current_conflict = cv2.line(image_current_conflict, pt_pix_A, pt_pix_B, (255, 0, 0), 2)

            # 标 TTC时间、冲突类型
            # 缩放大小0.8 蓝色，线宽 1
            if TTC_label :

                image_current_conflict = cv2.putText(image_current_conflict, 'TTC:'+ str(round(float(self._TTC), 2)) + 's',\
                                                     pt_conflict, cv2.FONT_HERSHEY_PLAIN, 2.5, (255, 0, 0), 2)

            if conflict_type_label:

                conflict_type = ''
                # 设置位置在TCC下面一行 （y坐标下移10个像素点）
                pt_text = cam_model.project_points(np.array([(self._x + 2, self._y - 0.5, 0.0)]))
                pt_text = (int(pt_text[0]), int(pt_text[1]))
                # 根据类型字符串，换成中文，
                if self._conflict_type == 'lateral_conflict':
                    conflict_type = 'lateral conflict'
                if self._conflict_type == 'A3_B12_rear_conflict':
                    conflict_type = 'rear-end conflict A3B12'
                if self._conflict_type == 'B2_A34_rear_conflict':
                    conflict_type = 'rear-end conflict B2A34'

                image_current_conflict = cv2.putText(image_current_conflict, conflict_type, pt_text, \
                                                     cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 0, 0), 1)

        return image_current_conflict




