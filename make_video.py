import argparse
import os
import sys
import shutil

import ffmpeg


def main(args_input):

    argparser = argparse.ArgumentParser(
        description='Rename images for making video')
    argparser.add_argument(
        '-video_img_path',
        default='',
        help='Path of the images')
    argparser.add_argument(
        '-origin_img_path',
        default='',
        help='Path of the origin images')
    argparser.add_argument(
        '-result_video_path',
        default='',
        help='Path of the origin images')

    args = argparser.parse_args(args_input)

    make_video(args)


def make_video(config):
    # file_dir = r'C:\Users\Fred\PycharmProjects\trajectory-extractor\mytest\test19\output\visualizer\make_video' # 原始字符串
    # file_dir = r'G:\trajectory-extractor\mytest\test19\output\visualizer\make_video'  # 原始字符串
    file_dir = config.video_img_path
    origin_img = config.origin_img_path
    # 创建make_video文件夹
    os.makedirs(file_dir, exist_ok=True)
    # 拷贝 原视频图片文件 到 当前文件夹
    for file in os.listdir(origin_img):
        elment_file_path = os.path.join(origin_img, file)
        if os.path.isfile(elment_file_path):
            shutil.copy(elment_file_path, file_dir)

    os.chdir(file_dir)  # 改变工作目录
    elment = os.listdir(file_dir)
    count = 0
    for i in elment:
        if os.path.isfile(i):
            remanefile = 'img{}.png'.format(count)
            os.rename(i, remanefile)
            count = count + 1
    print('copy img done!')

    # 在cmd中执行ffmpeg由图片生成视频（需要提前安装好ffmpeg并设置环境变量）
    os.system('ffmpeg -framerate 13 -i img%d.png -y out.mp4')
    origin_video = os.path.join(file_dir, 'out.mp4')
    dst_dir = os.path.join(config.result_video_path, '冲突检测.mp4')
    # 将生成的视频剪切到目标文件夹
    shutil.move(origin_video, dst_dir)


if __name__ == '__main__':

    try:
        main(sys.argv[1:])
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')