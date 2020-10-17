
import sys

from PyQt5.QtCore import Qt, QSize, QUrl

import View
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from traj_ext.visualization.run_visualizer import run_visualize_traj
import argparse
from PyQt5.QtGui import QPixmap, QImage
from PIL import Image
import cv2
from Thread import *

class MainCode(QWidget, View.Ui_MainWindow): # 继承界面类，实现界面、控制器分离
    def __init__(self):
        QWidget.__init__(self)
        # 加载界面
        View.Ui_MainWindow.__init__(self)
        self.setupUi(self)
        #self.frame_train.setVisible(False)
        #self.frame_apply.setVisible(False)
        #self.train_state=0
        #self.train.clicked.connect(self.show_train)
        #self.apply.clicked.connect(self.show_apply)
        #self.import_data.clicked.connect(self.showDialog)
        #self.import_data1.clicked.connect(self.showDialog)
        #self.start_train.clicked.connect(self.trainthread)
        #self.evaluate.clicked.connect(self.evaluation)
        #self.start_apply.clicked.connect(self.startapply)

        # 槽函数 = controller控制器，实现接收数据进行处理的功能
        # 在这里定义控制器
        self.pushButton.clicked.connect(self.logic_run_visualizer)
        self.stopButton.clicked.connect(self.stop)
        self.pauseButton.clicked.connect(self.pause)
        self.resumeButton.clicked.connect(self.resume)
        self.folderSelectButton.clicked.connect(self.folderSelect)
        self.videoSelectButton.clicked.connect(self.videoSelect)
        self.openResultButton.clicked.connect(self.openResult)
        # self.pushButton_2.clicked.connect(self.pic_show)

    # 打开提取结果文件夹
    def openResult(self):
        folder = os.getcwd() + r'\output\vehicles\traj\csv'
        os.system("start explorer " + folder)

    # 添加实时信息
    def addToreal_time_info(self, text):
        self.real_time_info.insertHtml("<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; \
                    margin-right:0px; -qt-block-indent:0; text-indent:0px;\">{}<br></p>".format(text))
        self.real_time_info.verticalScrollBar().setValue(self.real_time_info.verticalScrollBar().maximum())

    def videoSelect(self):
        file_name  = QFileDialog.getOpenFileName(self, "选择视频源文件", "./", "All Files (*);;Text Files (*.txt)")
        video_path = file_name[0]
        if (video_path == ""):
            QMessageBox.information(self, "提示", self.tr("没有选择视频文件！"))
        self.textEdit_2.setText(video_path)
        # 设置提取命名
        url = QUrl.fromLocalFile(video_path)
        self.textEdit_3.setText(re.split('\.', url.fileName())[0])
        # print(video_path)

    def folderSelect(self):
        directory = QFileDialog.getExistingDirectory(self, "选择工作空间件夹", "./")
        if (directory == ""):
            QMessageBox.information(self, "提示", self.tr("没有选择文件夹！"))
        else:
            self.textEdit.setText(directory)
            os.chdir(directory)
        # print(directory)

    def stop(self):
        self.stopButton.setEnabled(False)
        self.resumeButton.setEnabled(False)
        self.pauseButton.setEnabled(False)
        self.enableEverything()
        self.Thread.kill = True # 状态变量用于控制结束while true循环
        self.Thread.terminate()

    # 继续检测
    def resume(self):
        self.Thread.pause = False
        self.pauseButton.setEnabled(True)
        self.resumeButton.setEnabled(False)

    # 暂停
    def pause(self):
        self.pauseButton.setEnabled(False)
        # 终止线程
        self.Thread.pause = True
        # 恢复按钮、文本框
        self.resumeButton.setEnabled(True)
        # self.enableEverything()
        self.stopButton.setEnabled(True)


    def dispImage(self, image_concat_path):
        pixmap = QPixmap(image_concat_path)
        h = self.imgDisplay.height()
        w = self.imgDisplay.width()
        self.imgDisplay.setPixmap(pixmap.scaled(QSize(2 * h, 2 * w), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def pic_show(self):
        # 点击按钮选择文件并提取路径
        file_name = QFileDialog.getOpenFileName(self, "Open File", "./", "jpg (*.jpg)")
        image_path = file_name[0]
        if (file_name[0] == ""):
            QMessageBox.information(self, "提示", self.tr("没有选择图片文件！"))
        print(image_path)
        # 展示图片
        pixmap = QPixmap(image_path)
        h = self.imgDisplay.height()
        w = self.imgDisplay.width()
        self.imgDisplay.setPixmap(pixmap.scaled(QSize(h, w), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def enableEverything(self):
        self.pushButton.setEnabled(True)
        self.textEdit.setEnabled(True)
        self.textEdit_2.setEnabled(True)
        self.textEdit_3.setEnabled(True)
        self.videoSelectButton.setEnabled(True)
        self.folderSelectButton.setEnabled(True)
        self.openResultButton.setEnabled(True)

    def disableEverything(self):
        self.pushButton.setEnabled(False)
        self.textEdit.setEnabled(False)
        self.textEdit_2.setEnabled(False)
        self.textEdit_3.setEnabled(False)
        self.resumeButton.setEnabled(False)
        self.videoSelectButton.setEnabled(False)
        self.folderSelectButton.setEnabled(False)
        self.openResultButton.setEnabled(False)

    # 控制器：实现读取输入数据，调用函数进行处理
    def logic_run_visualizer(self):
        # 获取设置参数，判断参数非空
        # 执行可视化脚本
            # 读取输入数据
        SOURCE_FOLDER = self.textEdit.toPlainText()
        VIDEO_NAME = self.textEdit_2.toPlainText()
        NAME = self.textEdit_3.toPlainText()
        if (SOURCE_FOLDER != '') & (VIDEO_NAME != '') & (NAME != ''):

            # 实例化线程
            self.setThread(Thread())
            # 将信号的线程与对应的控制器函数连接
            self.Thread.imgSig.connect(self.dispImage)
            self.Thread.canvasSig.connect(self.addToreal_time_info)
            # 启动线程。进行监听！！！
            self.Thread.start()

            # SET THE PATH AND CONFIG
            VIDEO_PATH = SOURCE_FOLDER + '/' + VIDEO_NAME
            DELTA_MS = 100
            LOCATION_NAME = "shanghai"
            DATE = "20200604"
            START_TIME = "1210"

            CAMERA_STREET = "area1_street_cfg.yml"
            CAMERA_SAT = "sat_cfg.yml"
            CAMERA_SAT_IMG = "sat.png"
            HD_MAP = "area1_street_hd_map.csv"

            # SET DETECTION AD IGNORE ZONES
            DET_ZONE_IM_VEHICLES = NAME + '_detection_zone_im.yml'
            DET_ZONE_FNED_VEHICLES = NAME + '_detection_zone.yml'
            IGNORE_AREA_VEHICLES = ''

            DET_ZONE_IM_PEDESTRIANS = NAME + "_detection_zone_im.yml"
            DET_ZONE_FNED_PEDESTRIANS = NAME + "_detection_zone.yml"
            IGNORE_AREA_PEDESTRIANS = ""

            # SET CROP VALUES FOR DETECTION HERE IF NEEDED
            CROP_X1 = 180
            CROP_Y1 = 120
            CROP_X2 = 1250
            CROP_Y2 = 720

            IMG_OUTPUT_FOLDER = SOURCE_FOLDER
            TIME_MAX_S = "20"
            SKIP = "2"
            IMG_DIR = SOURCE_FOLDER + "/img"
            OUTPUT_DIR = SOURCE_FOLDER + "/output"

            DET_DIR = OUTPUT_DIR + "/det/csv"

            MODE_VEHICLES = "vehicles"
            DYNAMIC_MODEL_VEHICLES = "BM2"
            LABEL_REPLACE_VEHICLES = "car"
            OUTPUT_VEHICLES_DIR = OUTPUT_DIR + '/' + MODE_VEHICLES
            DET_ASSO_VEHICLES_DIR = OUTPUT_VEHICLES_DIR + "/det_association/csv"
            TRAJ_VEHICLES_DIR = OUTPUT_VEHICLES_DIR + "/traj/csv"
            TRACK_MERGE_VEHICLES = OUTPUT_VEHICLES_DIR + "/det_association/" + NAME + "_tracks_merge.csv"
            TRAJ_VEHICLES = TRAJ_VEHICLES_DIR + '/' + NAME + "_traj.csv"
            TRAJ_INSPECT_VEHICLES_DIR = OUTPUT_VEHICLES_DIR + "/traj_inspect/csv"
            TRAJ_INSPECT_VEHICLES = TRAJ_INSPECT_VEHICLES_DIR + '/' + NAME + "_traj.csv"
            TRAJ_INSPECT_VEHICLES_TIME = TRAJ_INSPECT_VEHICLES_DIR + '/' + NAME + "_time_traj.csv"

            TRAJ_NOT_MERGED_CSV = TRAJ_INSPECT_VEHICLES_DIR + '/' + NAME + "_traj_not_merged.csv"
            SHRINK_ZONE = 1
            MIN_LENGTH = 6

            # 按照函数参数格式，将想设置的参数编写为 命名空间
            parser = argparse.ArgumentParser(description='Visualize the final trajectories')
            parser.add_argument('-traj', default='')
            parser.add_argument('-image_dir', type=str, default='')
            parser.add_argument('-camera_street', type=str, default='')
            parser.add_argument('-camera_sat', type=str, default='')
            parser.add_argument('-camera_sat_img', type=str, default='')
            parser.add_argument('-det_zone_fned', type=str, default='')
            parser.add_argument('-hd_map', type=str, default='')
            parser.add_argument('-output_dir', type=str, default='')
            parser.add_argument('-time', default='')
            parser.add_argument('-traj_person', default='')
            parser.add_argument('-shrink_zone', type=float, default=1.0)
            parser.add_argument('-no_label', action='store_true')
            parser.add_argument('-export', type=bool, default=False)
            args = parser.parse_args(['-traj', TRAJ_INSPECT_VEHICLES, \
                                      '-image_dir', SOURCE_FOLDER + "/img", \
                                      '-camera_street', SOURCE_FOLDER + '/' + CAMERA_STREET, \
                                      '-camera_sat', SOURCE_FOLDER + '/' + CAMERA_SAT, \
                                      '-camera_sat_img', SOURCE_FOLDER + '/' + CAMERA_SAT_IMG, \
                                      '-det_zone_fned', SOURCE_FOLDER + '/' + DET_ZONE_FNED_VEHICLES, \
                                      '-hd_map', SOURCE_FOLDER + '/' + HD_MAP, \
                                      '-output_dir', OUTPUT_DIR,\
                                      # 设置了自动导出图片，如果需要手动控制，可以去掉，并在run_visualizer.py加上cv2.imshow()展示视图才能实现控制
                                      '-export', 'True'])
            # 输出看一下结果
            # print(args)
            # “停止”按钮恢复
            self.pauseButton.setEnabled(True)
            # 让“开始”按钮和文本框 disable
            self.disableEverything()
            # 再调用 run_visualize_traj（args）实现可视化
            run_visualize_traj(args, self.Thread)# 要定义线程,否则不能显示

        else:
            QMessageBox.information(self, "提示", self.tr("请输入完整信息！"))



if __name__=='__main__':
    app=QApplication(sys.argv)
    md=MainCode()
    md.show()
    sys.exit(app.exec_())
