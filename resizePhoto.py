import argparse
import os
import cv2
import sys

from pandas import np


def main(args_input):

    argparser = argparse.ArgumentParser(
        description='Resize images')
    argparser.add_argument(
        '-img_path',
        default='',
        help='Path of the images')

    args = argparser.parse_args(args_input)

    resize_photo(args)

def resize_photo(config):
    file_dir = config.img_path
    os.chdir(file_dir)  # 改变工作目录
    # 调整大小为50%
    for file in os.listdir(file_dir):
        elment_file_path = os.path.join(file_dir, file)
        if os.path.isfile(elment_file_path):
            im = cv2.imread(elment_file_path)
            pic = cv2.resize(src = im, dsize = (960, 540), interpolation = cv2.INTER_CUBIC)
            cv2.imwrite(elment_file_path, pic)
            print("Resize img{}".format(elment_file_path))
    print("Resize Done!")

if __name__ == '__main__':

    try:
        main(sys.argv[1:])
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
