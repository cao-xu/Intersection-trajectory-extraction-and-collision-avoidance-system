# -*- coding: utf-8 -*-
# @Author: Aubrey
# @Date:   2019-06-27 13:58:53
# @Last Modified by:   Aubrey
# @Last Modified time: 2019-10-27 22:29:12

import copy
import cv2
import argparse
import os
import subprocess
import math;
import sys
import json
import time

from conflict_detection.conflict import Conflict
from traj_ext.postprocess_track import trajutil
from traj_ext.postprocess_track.trajectory import Trajectory

from traj_ext.tracker.cameramodel import CameraModel

from traj_ext.postprocess_track.agent_type_correct import AgentTypeCorrect

from traj_ext.object_det.det_object import DetObject
from traj_ext.object_det.mask_rcnn import detect_utils

from traj_ext.visualization import run_inspect_traj
from traj_ext.hd_map.HD_map import HDmap


from traj_ext.utils import det_zone
from traj_ext.utils import mathutil

from traj_ext.tracker import EKF_utils


def main(args_input):

    # Print instructions
    print("############################################################")
    print("Visualize the final trajectories")
    print("############################################################\n")

    # ##########################################################
    # # Parse Arguments
    # ##########################################################
    argparser = argparse.ArgumentParser(
        description='Visualize the final trajectories')
    argparser.add_argument(
        '-traj',
        default='',
        help='Path of the trajectories csv')
    # 冲突点文件路径
    argparser.add_argument(
        '-conflicts',
        default='',
        help='Path of the conflicts csv')
    argparser.add_argument(
        '-traj_person',
        default='',
        help='Path of the person trajectories csv')
    argparser.add_argument(
        '-time',
        default='',
        help='Path of the time csv')

    argparser.add_argument(
        '-image_dir',
        type=str,
        default='',
        help='Path of the image folder')
    argparser.add_argument(
        '-camera_street',
        type=str,
        default='',
        help='Path to the camera street model yaml');
    argparser.add_argument(
        '-camera_sat',
        type=str,
        default='',
        help='Path to the camera sat model yaml');
    argparser.add_argument(
        '-camera_sat_img',
        type=str,
        default='',
        help='Path to the camera sat image');
    argparser.add_argument(
        '-det_zone_fned',
        type=str,
        default='',
        help='Path of the detection zone fned file');
    argparser.add_argument(
        '-shrink_zone',
        type = float,
        default = 1.0,
        help='Detection zone shrink coefficient for complete trajectories');
    argparser.add_argument(
        '-hd_map',
        type = str,
        default = '',
        help='Path to the HD map');
    argparser.add_argument(
        '-no_label',
        action ='store_true',
        help='Do not display track id');

    argparser.add_argument(
        '-output_dir',
        type=str,
        default='',
        help='Path of the output');

    argparser.add_argument(
        '-export',
        type=bool,
        default = False,
        help='Export trajectories directly and exit the program (for automated process)');

    argparser.add_argument(
        '-config_json',
        default='',
        help='Path to json config')

    args = argparser.parse_args(args_input);

    if os.path.isfile(args.config_json):
        with open(args.config_json, 'r') as f:
            data_json = json.load(f)
            vars(args).update(data_json)

    vars(args).pop('config_json', None);

    # run_visualize_traj(args);
    run_visualize_conflict_and_traj(args)

def run_visualize_traj(config, Thread = None): # 模仿yolo_gpu_bugged.py加入 thread 参数，关注是否存在进程

    # Create output folder
    output_dir = config.output_dir;
    output_dir = os.path.join(output_dir, 'visualizer');
    os.makedirs(output_dir, exist_ok=True)

    # Create output sub-folder
    image_raw_saving_dir = os.path.join(output_dir, 'img_raw');
    image_annoted_saving_dir = os.path.join(output_dir, 'img_annoted');
    image_hd_map_saving_dir = os.path.join(output_dir, 'img_hdmap');
    image_concat_saving_dir = os.path.join(output_dir, 'img_concat');

    os.makedirs(image_raw_saving_dir, exist_ok=True)
    os.makedirs(image_annoted_saving_dir, exist_ok=True)
    os.makedirs(image_hd_map_saving_dir, exist_ok=True)
    os.makedirs(image_concat_saving_dir, exist_ok=True)

    # Save the cfg file with the output:
    try:
        cfg_save_path = os.path.join(output_dir, 'visualize_traj_cfg.json');
        with open(cfg_save_path, 'w') as json_file:
            config_dict = vars(config);
            json.dump(config_dict, json_file, indent=4)
    except Exception as e:
        print('[ERROR]: Error saving config file in output folder:\n')
        print('{}'.format(e))
        return;

    #Person trajectories
    traj_person_list = [];
    traj_person_available = os.path.isfile(config.traj_person);
    if traj_person_available:
        traj_person_list = Trajectory.read_trajectory_panda_csv(config.traj_person);

    # Open objects
    cam_model = CameraModel.read_from_yml(config.camera_street);
    det_zone_FNED = det_zone.DetZoneFNED.read_from_yml(config.det_zone_fned);

    # Time
    list_times_ms_path = config.time;
    if list_times_ms_path == '':
        list_times_ms_path = config.traj.replace('traj.csv', 'time_traj.csv');
    if not os.path.isfile(list_times_ms_path):
        print('[ERROR]: Traj time file not found: {}'.format(list_times_ms_path))
        return;
    list_times_ms = trajutil.read_time_list_csv(list_times_ms_path);

    # Trajectories
    traj_list = Trajectory.read_trajectory_panda_csv(config.traj);

    sat_view_available = False;
    sat_view_enable = False;
    if os.path.isfile(config.camera_sat) and os.path.isfile(config.camera_sat_img):
        cam_model_sat = CameraModel.read_from_yml(config.camera_sat);
        image_sat = cv2.imread(os.path.join(config.camera_sat_img));

        sat_view_available = True;

    # Check if image is directoty
    image_in_dir = os.path.isdir(config.image_dir);

    # If image directory, open list of images
    if image_in_dir:
        list_img_file = os.listdir(config.image_dir);
        list_img_file.sort(key=lambda f: int(''.join(filter(str.isdigit, f))));

    # Else open the unique image
    else:
        image = cv2.imread(config.image_dir);

    hd_map_available = os.path.isfile(config.hd_map);
    if hd_map_available:

        hd_map = HDmap.from_csv(config.hd_map);

        cam_model_hdmap, image_hdmap = hd_map.create_view();

        image_hdmap = hd_map.display_on_image(image_hdmap, cam_model_hdmap);


    # Shrink det zone for complete
    det_zone_FNED_complete = None;
    if config.shrink_zone < 1:
        det_zone_FNED_complete = det_zone_FNED.shrink_zone(config.shrink_zone);
        for traj in traj_list:
            if not traj.check_is_complete(det_zone_FNED_complete):
                print('Track: {} time_ms: {} not complete'.format(traj.get_id(), traj.get_start_trajoint().time_ms))



    skip_value = 1;
    frame_index = 0;
    export_mode = False;

    # If export mode is direclty asked
    if config.export:
        export_mode = True;

    while True:

        #######################################################
        ## Show Trajectories
        #######################################################

        # Get Time ms
        frame_index = mathutil.clip(frame_index, 0, len(list_times_ms)-1); #剪辑返回的是帧序，保证帧序在0到最大之间
        time_ms = list_times_ms[frame_index]; # 读取当前的时刻，用于遍历所有存在轨迹的时间段

        if image_in_dir:
            img_file_name = list_img_file[frame_index];
            image_current = cv2.imread(os.path.join(config.image_dir, img_file_name));
        else:
            # Copy image
            image_current = image;


        if (not (image_current is None)) & (not Thread.pause): # 加上 (not Thread.pause)是为了在暂停时也停下画图

            print('Showing: frame_id: {} image: {}'.format(frame_index, img_file_name));
            Thread.signalCanvas("Showing: frame_id: {} image: {}".format(frame_index, img_file_name))

            # Display traj
            ################# 改为展示从0到当前time_ms所有点画在图上
            #  要实现速度标签打上去，加上“velocity_label = True”

            image_current_traj, _ = run_inspect_traj.display_traj_on_image(time_ms, cam_model, image_current, traj_list, det_zone_FNED_list = [det_zone_FNED], no_label = config.no_label, velocity_label = True);
            image_current_traj, _ = run_inspect_traj.display_traj_on_image(time_ms, cam_model, image_current_traj, traj_person_list, det_zone_FNED_list = [det_zone_FNED], no_label = config.no_label, velocity_label = True);

            # 在图片上画出轨迹线？
            # Show image
            # cv2.imshow('Trajectory visualizer', image_current_traj)

            # if sat_view_available:
            #
            #     if sat_view_enable:
            #
            #         # Display traj
            #         image_sat_current, _ = run_inspect_traj.display_traj_on_image(time_ms, cam_model_sat, image_sat, traj_list, det_zone_FNED_list = [det_zone_FNED], no_label = config.no_label);
            #         image_sat_current, _ = run_inspect_traj.display_traj_on_image(time_ms, cam_model_sat, image_sat_current, traj_person_list, det_zone_FNED_list = [det_zone_FNED], no_label = config.no_label)
            #         # Show image
            #         cv2.imshow('Sat View merged', image_sat_current)

            if hd_map_available:

                # Display traj
                image_hdmap_current, _ = run_inspect_traj.display_traj_on_image(time_ms, cam_model_hdmap, image_hdmap, traj_list, det_zone_FNED_list = [det_zone_FNED], no_label = config.no_label, velocity_label=True);
                image_hdmap_current, _ = run_inspect_traj.display_traj_on_image(time_ms, cam_model_hdmap, image_hdmap_current, traj_person_list, det_zone_FNED_list = [det_zone_FNED], no_label = config.no_label, velocity_label=True);

                # # Show image
                # cv2.imshow('HD map view merged', image_hdmap_current)


                image_concat = EKF_utils.concatenate_images(image_current_traj, image_hdmap_current)
                # 隐藏了结合视图
                # cv2.imshow('View: Camera and HD map', image_concat)

            if export_mode:

                img_annoted_name = img_file_name.split('.')[0] + '_annotated.png';
                print('Saving: frame_id: {} image: {}'.format(frame_index, img_annoted_name))

                image_annoted_path = os.path.join(image_annoted_saving_dir, img_annoted_name);
                cv2.imwrite(image_annoted_path, image_current_traj);

                print('Saving: frame_id: {} image: {}'.format(frame_index, img_file_name))
                image_raw_path = os.path.join(image_raw_saving_dir, img_file_name);
                cv2.imwrite(image_raw_path, image_current);

                if hd_map_available:

                    img_hdmap_name = img_file_name.split('.')[0] + '_hdmap.png';
                    print('Saving: frame_id: {} image: {}'.format(frame_index, img_hdmap_name))
                    image_hdmap_path = os.path.join(image_hd_map_saving_dir, img_hdmap_name);
                    cv2.imwrite(image_hdmap_path, image_hdmap_current);

                    img_concat_name = img_file_name.split('.')[0] + '_concat.png';
                    print('Saving: frame_id: {} image: {}'.format(frame_index, img_concat_name))
                    Thread.signalCanvas('Saving: frame_id: {} image: {}'.format(frame_index, img_concat_name))
                    image_concat_path = os.path.join(image_concat_saving_dir, img_concat_name);
                    cv2.imwrite(image_concat_path, image_concat);
                    # 发送信号给meau_controller，在窗口实现图片展示
                    if not Thread.pause: # 线程未暂停

                        Thread.signalImg(image_concat_path)
                    # 如何实现线程的暂停与再次启动


        elif Thread.pause: # 若是因为暂停
            a = 1
            # print('暂停检测')
            # time.sleep(3)
        else:
            print('Not found: frame_id: {} image: {}'.format(frame_index, img_file_name));

        if Thread.kill: # 若点击“结束”，则结束循环
            break

        #######################################################
        ## Control keys
        #######################################################

        if frame_index == (len(list_times_ms)-1):
            print('Export view: Done');
            Thread.signalCanvas('Export view: Done')
            export_mode = False;

            # Exit when it is done if in export directly mode
            if config.export:
                break;

        wait_key = 0;
        if export_mode & (not Thread.pause): # 暂停，不再读取下一帧
            frame_index +=1;
            wait_key = 1;

        key = cv2.waitKey(wait_key) & 0xFF

        if key == ord("n"):
            frame_index +=skip_value;
            mathutil.clip(frame_index, 0, len(list_times_ms));

        elif key == ord("b"):
            frame_index +=1000*skip_value;
            mathutil.clip(frame_index, 0, len(list_times_ms));

        elif  key == ord("p"):
            frame_index -=skip_value;
            mathutil.clip(frame_index, 0, len(list_times_ms));

        elif  key == ord("o"):
            frame_index -=1000*skip_value;
            mathutil.clip(frame_index, 0, len(list_times_ms));

        # Escape: Quit the program
        elif key == 27:
            break;

        # Escape: Quit the program
        elif key == ord('z'):
            if sat_view_available:
                if sat_view_enable:
                    cv2.destroyWindow('Sat View merged');

                    sat_view_enable = False;

                else:
                    sat_view_enable = True;

        elif key == ord("+"):
            skip_value +=1;
            skip_value = max(1, skip_value);
            print('Skip value: {}'.format(skip_value))

        elif key == ord("-"):
            skip_value -=1;
            skip_value = max(1, skip_value);
            print('Skip value: {}'.format(skip_value))

        elif key == ord("e"):

            if not export_mode:
                export_mode = True;
                frame_index = 0;
                print('Mode: Export mode started');

            else:
                export_mode = False;
                print('Mode: Export mode stopped');

        elif key == 255:
            pass;

        else:

            print('\nInstruction:\n- n: Next frame\
                                 \n- b: Jump 1000 frame forward\
                                 \n- p: Previous frame\
                                 \n- b: Jump 1000 frame backward\
                                 \n- +: Increase skip value\
                                 \n- -: Decrease skip value\
                                 \n- d: Open detection window\
                                 \n- c: Open Agent type correction\
                                 \n- m: Open merging file with sublime text\
                                 \n- s: Enable saving form current frame\
                                 \n- Click on Detection window: Enable/disable detections\
                                 \n- f: Display only complete trajectories\
                                 \n- i: Open ignore trajectory file\
                                 \n- e: Export trajectory file\
                                 \n- esc: Quit\
                                 \n')

# 画 轨迹 的同时画 冲突点
def print_conflicts_result(conflict_list, Thread):
    if len(conflict_list) != 0:

        Thread.signalCanvas("##########################################################################")
        Thread.signalCanvas("-----------------------------冲突检测结果-------------------------------")
        Thread.signalCanvas("##########################################################################")
        Thread.signalCanvas("|   时刻(ms)   |  TTC(s)  |  冲突车辆_1_ID(#)  |  冲突车辆_2_ID(#)  |")
        # 修改输出属性

        for per_ms_conflict_list in conflict_list:

            for conflict_point in per_ms_conflict_list:
                Thread.signalCanvas("| __{}__ | __{}__ | __{}__ | __{}__ |".format(int(conflict_point.get_time_ms()), \
                                                                   round(conflict_point.get_TTC(), 2), \
                                                                   int(conflict_point.get_track_id_A()), \
                                                                   int(conflict_point.get_track_id_B())))
        Thread.signalCanvas("##########################################################################")


def run_visualize_conflict_and_traj(config, Thread = None): # 模仿yolo_gpu_bugged.py加入 thread 参数，关注是否存在进程

    # Create output folder
    output_dir = config.output_dir;
    output_dir = os.path.join(output_dir, 'visualizer');
    os.makedirs(output_dir, exist_ok=True)

    # Create output sub-folder
    image_raw_saving_dir = os.path.join(output_dir, 'img_raw')
    image_annoted_saving_dir = os.path.join(output_dir, 'img_annoted')
    image_hd_map_saving_dir = os.path.join(output_dir, 'img_hdmap')
    image_concat_saving_dir = os.path.join(output_dir, 'img_concat')
    # 用于保存画了 冲突点 的图片
    image_concat_with_conflict_point_saving_dir = os.path.join(output_dir, 'img_concat_with_conflict')

    os.makedirs(image_raw_saving_dir, exist_ok=True)
    os.makedirs(image_annoted_saving_dir, exist_ok=True)
    os.makedirs(image_hd_map_saving_dir, exist_ok=True)
    os.makedirs(image_concat_saving_dir, exist_ok=True)
    os.makedirs(image_concat_with_conflict_point_saving_dir, exist_ok=True)

    # Save the cfg file with the output:
    try:
        cfg_save_path = os.path.join(output_dir, 'visualize_traj_cfg.json');
        with open(cfg_save_path, 'w') as json_file:
            config_dict = vars(config);
            json.dump(config_dict, json_file, indent=4)
    except Exception as e:
        print('[ERROR]: Error saving config file in output folder:\n')
        print('{}'.format(e))
        return;

    #Person trajectories
    traj_person_list = [];
    traj_person_available = os.path.isfile(config.traj_person);
    if traj_person_available:
        traj_person_list = Trajectory.read_trajectory_panda_csv(config.traj_person);

    # Open objects
    cam_model = CameraModel.read_from_yml(config.camera_street);
    det_zone_FNED = det_zone.DetZoneFNED.read_from_yml(config.det_zone_fned);

    # Time
    list_times_ms_path = config.time;
    if list_times_ms_path == '':
        list_times_ms_path = config.traj.replace('traj.csv', 'time_traj.csv');
    if not os.path.isfile(list_times_ms_path):
        print('[ERROR]: Traj time file not found: {}'.format(list_times_ms_path))
        return;
    list_times_ms = trajutil.read_time_list_csv(list_times_ms_path);

    # Trajectories
    traj_list = Trajectory.read_trajectory_panda_csv(config.traj)

    # 读取Conflict_list
    conflict_list = Conflict.read_conflict_panda_csv(config.conflicts)

    sat_view_available = False;
    sat_view_enable = False;
    if os.path.isfile(config.camera_sat) and os.path.isfile(config.camera_sat_img):
        cam_model_sat = CameraModel.read_from_yml(config.camera_sat);
        image_sat = cv2.imread(os.path.join(config.camera_sat_img));

        sat_view_available = True;

    # Check if image is directoty
    image_in_dir = os.path.isdir(config.image_dir);

    # If image directory, open list of images
    if image_in_dir:
        list_img_file = os.listdir(config.image_dir);
        list_img_file.sort(key=lambda f: int(''.join(filter(str.isdigit, f))));

    # Else open the unique image
    else:
        image = cv2.imread(config.image_dir);

    hd_map_available = os.path.isfile(config.hd_map);
    if hd_map_available:

        hd_map = HDmap.from_csv(config.hd_map);

        cam_model_hdmap, image_hdmap = hd_map.create_view();

        image_hdmap = hd_map.display_on_image(image_hdmap, cam_model_hdmap);


    # Shrink det zone for complete
    det_zone_FNED_complete = None;
    if config.shrink_zone < 1:
        det_zone_FNED_complete = det_zone_FNED.shrink_zone(config.shrink_zone);
        for traj in traj_list:
            if not traj.check_is_complete(det_zone_FNED_complete):
                print('Track: {} time_ms: {} not complete'.format(traj.get_id(), traj.get_start_trajoint().time_ms))



    skip_value = 1;
    frame_index = 0;
    export_mode = False;

    # If export mode is direclty asked
    if config.export:
        export_mode = True;

    while True:

        #######################################################
        ## Show Trajectories And Conflict
        #######################################################

        # Get Time ms
        frame_index = mathutil.clip(frame_index, 0, len(list_times_ms)-1); #剪辑返回的是帧序，保证帧序在0到最大之间
        time_ms = list_times_ms[frame_index]; # 读取当前的时刻，用于遍历所有存在轨迹的时间段

        if image_in_dir:
            img_file_name = list_img_file[frame_index];
            image_current = cv2.imread(os.path.join(config.image_dir, img_file_name));
        else:
            # Copy image
            image_current = image;


        if (not (image_current is None)) & (not Thread.pause): # 加上 (not Thread.pause)是为了在暂停时也停下画图

            print('Showing: frame_id: {} image: {}'.format(frame_index, img_file_name));
            Thread.signalCanvas("Showing: frame_id: {} image: {}".format(frame_index, img_file_name))

            # Display traj
            ################# 改为展示从0到当前time_ms所有点画在图上
            #  要实现速度标签打上去，加上“velocity_label = True”
            image_current_traj, _ = run_inspect_traj.display_conflict_traj_on_image(time_ms, cam_model, image_current, traj_list, det_zone_FNED_list = [det_zone_FNED], no_label = config.no_label, velocity_label = True);
            image_current_traj, _ = run_inspect_traj.display_conflict_traj_on_image(time_ms, cam_model, image_current_traj, traj_person_list, det_zone_FNED_list = [det_zone_FNED], no_label = config.no_label, velocity_label = True);

            # 在图片上画出轨迹线？
            # Show image
            # cv2.imshow('Trajectory visualizer', image_current_traj)

            # if sat_view_available:
            #
            #     if sat_view_enable:
            #
            #         # Display traj
            #         image_sat_current, _ = run_inspect_traj.display_traj_on_image(time_ms, cam_model_sat, image_sat, traj_list, det_zone_FNED_list = [det_zone_FNED], no_label = config.no_label);
            #         image_sat_current, _ = run_inspect_traj.display_traj_on_image(time_ms, cam_model_sat, image_sat_current, traj_person_list, det_zone_FNED_list = [det_zone_FNED], no_label = config.no_label)
            #         # Show image
            #         cv2.imshow('Sat View merged', image_sat_current)

            # 为了降低耦合性，分为两个函数写

            # Display Conflicts

            # 设置要画冲突点的图片
            image_traj_with_conflict = image_current_traj

            # 判断当前 整个视频 是否存在冲突点，若冲突数组（即冲突csv文件为空）不为空，继续
            if len(conflict_list) != 0:

                for per_ms_conflict_list in conflict_list:

                    # 如果当前时刻存在冲突数组，则循环画上去，画 点 和 线 以及标 TTC
                    if per_ms_conflict_list[0].get_time_ms() == time_ms:

                        # print('画' + str(time_ms) + 'ms 时刻冲突点\n\n\n')
                        for conflict_point in per_ms_conflict_list:

                            image_traj_with_conflict = conflict_point.display_conflict_point_on_image(time_ms, \
                                                                                                      cam_model, \
                                                                                                      image_traj_with_conflict, \
                                                                                                      traj_list, \
                                                                                                      TTC_label = True,\
                                                                                                      is_hd_map = False)
                            Thread.signalCanvas(str(time_ms) + "ms 时刻检测到冲突，TTC：{} s, 冲突车1的x坐标：{}，冲突车1的y坐标{}，"
                                                "冲突车2的x坐标：{}，冲突车1的y坐标{}\n"\
                                                .format(round(float(conflict_point.get_TTC()), 2), \
                                                        round(float(conflict_point.get_point_A_x()), 2),\
                                                        round(float(conflict_point.get_point_A_y()), 2),\
                                                        round(float(conflict_point.get_point_B_x()), 2),\
                                                        round(float(conflict_point.get_point_B_y()), 2)))

            if hd_map_available:

                # Display traj
                image_hdmap_current, _ = run_inspect_traj.display_traj_on_image(time_ms,\
                                                                                cam_model_hdmap,\
                                                                                image_hdmap,\
                                                                                traj_list,\
                                                                                det_zone_FNED_list = [det_zone_FNED],\
                                                                                no_label = config.no_label,\
                                                                                velocity_label=True)
                image_hdmap_current, _ = run_inspect_traj.display_traj_on_image(time_ms,\
                                                                                cam_model_hdmap,\
                                                                                image_hdmap_current,\
                                                                                traj_person_list,\
                                                                                det_zone_FNED_list = [det_zone_FNED],\
                                                                                no_label = config.no_label,\
                                                                                velocity_label=True)

                # # Show image
                # cv2.imshow('HD map view merged', image_hdmap_current)

                # 模仿上面的函数在 俯视 视角的图片中画 冲突点
                image_hd_traj_with_conflict = image_hdmap_current
                # 判断当前 整个视频 是否存在冲突点，若冲突数组（即冲突csv文件为空）不为空，继续
                if len(conflict_list) != 0:

                    for per_ms_conflict_list in conflict_list:

                        # 如果当前时刻存在冲突数组，则循环画上去，画 点 和 线 以及标 TTC
                        if per_ms_conflict_list[0].get_time_ms() == time_ms:

                            # print('画' + str(time_ms) + 'ms 时刻俯视冲突点\n\n\n')
                            for conflict_point in per_ms_conflict_list:

                                image_hd_traj_with_conflict = conflict_point.display_conflict_point_on_image(time_ms,\
                                                                                                             cam_model_hdmap,\
                                                                                                             image_hd_traj_with_conflict,\
                                                                                                             traj_list,\
                                                                                                             TTC_label=True, \
                                                                                                             is_hd_map = True)




                image_concat = EKF_utils.concatenate_images(image_traj_with_conflict, image_hd_traj_with_conflict)
                # 隐藏了结合视图
                # cv2.imshow('View: Camera and HD map', image_concat)

            if export_mode:

                img_annoted_name = img_file_name.split('.')[0] + '_annotated.png'
                print('Saving: frame_id: {} image: {}'.format(frame_index, img_annoted_name))

                image_annoted_path = os.path.join(image_annoted_saving_dir, img_annoted_name)
                cv2.imwrite(image_annoted_path, image_traj_with_conflict)

                print('Saving: frame_id: {} image: {}'.format(frame_index, img_file_name))
                image_raw_path = os.path.join(image_raw_saving_dir, img_file_name)
                cv2.imwrite(image_raw_path, image_current)

                if hd_map_available:

                    img_hdmap_name = img_file_name.split('.')[0] + '_hdmap.png'
                    print('Saving: frame_id: {} image: {}'.format(frame_index, img_hdmap_name))
                    image_hdmap_path = os.path.join(image_hd_map_saving_dir, img_hdmap_name)
                    cv2.imwrite(image_hdmap_path, image_hd_traj_with_conflict)

                    img_concat_name = img_file_name.split('.')[0] + '_concat.png'
                    print('Saving: frame_id: {} image: {}'.format(frame_index, img_concat_name))
                    Thread.signalCanvas('Saving: frame_id: {} image: {}'.format(frame_index, img_concat_name))
                    image_concat_path = os.path.join(image_concat_saving_dir, img_concat_name)
                    cv2.imwrite(image_concat_path, image_concat)
                    # 发送信号给meau_controller，在窗口实现图片展示
                    if not Thread.pause: # 线程未暂停

                        Thread.signalImg(image_concat_path)
                    # 如何实现线程的暂停与再次启动


        elif Thread.pause: # 若是因为暂停
            a = 1
            # print('暂停检测')
            # time.sleep(3)
        else:
            print('Not found: frame_id: {} image: {}'.format(frame_index, img_file_name));

        if Thread.kill: # 若点击“结束”，则结束循环
            break

        #######################################################
        ## Control keys
        #######################################################

        if frame_index == (len(list_times_ms)-1):
            print('Export view: Done');
            Thread.signalCanvas('Export view: Done')
            export_mode = False;
            # 展示所有 检测冲突数据
            print_conflicts_result(conflict_list, Thread)

            # Exit when it is done if in export directly mode
            if config.export:
                break;

        wait_key = 0;
        if export_mode & (not Thread.pause): # 暂停，不再读取下一帧
            frame_index +=1;
            wait_key = 1;

        key = cv2.waitKey(wait_key) & 0xFF

        if key == ord("n"):
            frame_index +=skip_value;
            mathutil.clip(frame_index, 0, len(list_times_ms));

        elif key == ord("b"):
            frame_index +=1000*skip_value;
            mathutil.clip(frame_index, 0, len(list_times_ms));

        elif  key == ord("p"):
            frame_index -=skip_value;
            mathutil.clip(frame_index, 0, len(list_times_ms));

        elif  key == ord("o"):
            frame_index -=1000*skip_value;
            mathutil.clip(frame_index, 0, len(list_times_ms));

        # Escape: Quit the program
        elif key == 27:
            break;

        # Escape: Quit the program
        elif key == ord('z'):
            if sat_view_available:
                if sat_view_enable:
                    cv2.destroyWindow('Sat View merged');

                    sat_view_enable = False;

                else:
                    sat_view_enable = True;

        elif key == ord("+"):
            skip_value +=1;
            skip_value = max(1, skip_value);
            print('Skip value: {}'.format(skip_value))

        elif key == ord("-"):
            skip_value -=1;
            skip_value = max(1, skip_value);
            print('Skip value: {}'.format(skip_value))

        elif key == ord("e"):

            if not export_mode:
                export_mode = True;
                frame_index = 0;
                print('Mode: Export mode started');

            else:
                export_mode = False;
                print('Mode: Export mode stopped');

        elif key == 255:
            pass;

        else:

            print('\nInstruction:\n- n: Next frame\
                                 \n- b: Jump 1000 frame forward\
                                 \n- p: Previous frame\
                                 \n- b: Jump 1000 frame backward\
                                 \n- +: Increase skip value\
                                 \n- -: Decrease skip value\
                                 \n- d: Open detection window\
                                 \n- c: Open Agent type correction\
                                 \n- m: Open merging file with sublime text\
                                 \n- s: Enable saving form current frame\
                                 \n- Click on Detection window: Enable/disable detections\
                                 \n- f: Display only complete trajectories\
                                 \n- i: Open ignore trajectory file\
                                 \n- e: Export trajectory file\
                                 \n- esc: Quit\
                                 \n')

if __name__ == '__main__':

    try:
        main(sys.argv[1:])
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')