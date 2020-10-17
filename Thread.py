from PyQt5.QtCore import QThread, pyqtSignal, Qt
import re
import os
import traceback

class Thread(QThread):
    imgSig = pyqtSignal(str)
    canvasSig = pyqtSignal(str)

    def __init__(self):
        QThread.__init__(self)
        self.pause = False
        self.kill = False


    def signalImg(self, image_concat_path): # 参数是要展示的图片的路径
        self.imgSig.emit(image_concat_path)

    def signalCanvas(self, txt):
        # pass
        self.canvasSig.emit(txt)
