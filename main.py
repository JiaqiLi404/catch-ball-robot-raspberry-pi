import cv2
import numpy as np
import serial
from operator import itemgetter
import pyzbar.pyzbar as pyzbar
import threading
import time

import RPi.GPIO as gpio


# ####################### 参数类 ######################## #
class Information():
    # 测试模式
    debug = True
    debug_single_camera = False
    pc = False

    # 抓取参数
    # 40cm高度手爪抓球点坐标
    tar40_x = 185
    tar40_y = 177  # TODO
    tar40_delta_x = 22  # TODO
    tar40_delta_y = 10  # TODO
    max_deftime = 3  # 抓取失败的最大次数
    def_movetime = 10  # 抓取失败后为了丢失目标进行前进的次数#TODO
    img_delay = 1  # 信号发送延时1#TODO
    img_read_delay = 15  # 信号发送延时2#TODO
    ball_block_threshold = 1.2  # 越大越容易判断为是矩形
    max_movetime = 30  # 当多次调整位置均找不到合适抓取点时直接抓
    check_fetch_threshold = 1000  # 判断是否抓取成功的阈值
    block_long_short_threshold = 1.3  # 判断积木长宽的阈值
    camera_arm_angle = 0.5  # 抓取点与摄像头的角度#TODO
    catch_delay = 0.8  # 抓动态球时手臂运动的时延#TODO

    # 图像处理参数
    # 滤波
    need_threshold = True
    threshold_core = 7
    threshild_th = 230
    # 找球
    use_hough = False
    ball_minradius = 10
    hough_ballsh = 120
    hough_dp = 3
    # 颜色阈值

    # 串口通讯参数
    if pc:
        serial_port = None
    else:
        serial_port = "/dev/ttyAMA0"
    serial_rate = 460800
    serial_timeout = 0.1

    # 通讯类接口
    # 接收接口
    # -1-错误状态，0-监听状态，1-判断己方颜色，2-判断圆盘球，3-取圆盘球，4-取立柱球，5-取条形平台球，6-取条形平台积木，7-放球
    begin_flags = {'b1': 1, 'b2': 2, 'b3': 3, 'b4': 4, 'b5': 5, 'b6': 6, 'b7': 7}
    ends_flag = 'end'
    getball_flag = 'get'
    # 发给主控接口
    movedown_flag = '#on'
    movedown_quick_flag = '#qo'
    moveup_flag = '#ba'
    moveright_flag = '#do'
    moveleft_flag = '#up'
    success_flag = '#su'
    # 发给机械臂接口
    unfound_flag = 'nof'
    fetch_ball_flag = 'ftb'
    fetch_long_flag = 'ftl'
    fetch_short_flag = 'fts'
    defect_flag = 'def'
    bingo_flag = 'bin'
    turn_bingo_flag = 'tbg'
    we_are_blue_flag = 'wab'
    we_are_red_flag = 'war'

    # 比赛信息
    ourcolor = None  # 无法提前知道
    ball_movable = False  # 是否用圆盘机/黄球
    block_usecode = False  # 积木是否有颜色/二维码

    # 摄像头参数
    top_camera_id = 0
    side_camera_id = 2
    if pc:
        img_w = 640
        img_h = 480
    else:
        img_w = 480
        img_h = 320

    # 超声波参数
    trig = 17
    echo = 18
    detect_time = 5
    ball_distance = 5  # TODO
    errtime = 5

    # 程序信息
    next_ball_color = None
    now_ball_color = None
    circle_time = None

    def set_ChuSai(self):
        self.ball_movable = False
        self.block_usecode = False

    def set_YuJueSai(self):
        self.ball_movable = True
        self.block_usecode = True


# ####################### 串口通讯类 ######################## #
class Communicator():
    def __init__(self, com, rate, timeout):
        if inf.pc:
            self.ser = None
            return
        # 搭建串口通讯环境
        self.ser = serial.Serial(com, rate, timeout=timeout)
        if (self.ser.isOpen == False):
            self.ser.open()
        print('端口打开:', com, ' 波率:', rate, ' 超时:', timeout)
        self.ser.flushInput()

    # 监听开始工作信号
    def listener_begin(self):
        if inf.pc:
            return

        beginflag = 0
        while not beginflag:
            if self.ser.inWaiting() != 0:
                try:
                    beginflag = inf.begin_flags[self.ser.readline().decode().rstrip('\n')]
                except BaseException:
                    print('收到错误信息:', self.ser.readline().decode())
                    return -1
            time.sleep(0.1)
        print('收到开始标志:', beginflag)
        return beginflag

    # 监听停止识别信号
    def listener_end(self):
        if inf.pc:
            return

        endflag = False
        if self.ser.inWaiting() != 0 and self.ser.readline().decode().find(inf.ends_flag) != -1:
            print('收到停止标志，进入监听状态')
            endflag = True
        return endflag

    # 监听抓球结束信号
    def listener_getball(self):
        if inf.pc:
            return

        endflag = False
        if self.ser.inWaiting() != 0 and self.ser.readline().decode().find(inf.getball_flag) != -1:
            print('收到抓取完成标志')
            endflag = True
        return endflag

    # 发送信息给单片机
    def writer(self, words):
        print('串口发送:', words)
        if inf.pc:
            return
        self.ser.write(words.encode())


# ####################### 超声波测距类 ######################## #
class Measurer():
    def __init__(self):
        gpio.setmode(gpio.BCM)
        gpio.setwarnings(False)
        gpio.setup(inf.trig, gpio.OUT)
        gpio.setup(inf.echo, gpio.IN)

    # 基础功能：读取距离
    def get_distance(self):
        gpio.output(inf.trig, True)
        time.sleep(0.00001)
        gpio.output(inf.trig, False)
        starttime = time.time()
        while (gpio.input(inf.echo) == 0):
            nowtime = time.time()
            if nowtime >= starttime + 0.2:
                print("timeout err")
                return None
        starttime = time.time()
        while (gpio.input(inf.echo) == 1):
            nowtime = time.time()
            if nowtime >= starttime + 0.2:
                print("no signal err")
                return None
        endtime = time.time()
        distance = round((endtime - starttime) * 17150, 1)
        print(distance, "\n")
        return distance

    # 获得球的平均间隔
    def get_aver_time(self):
        timecosts = []
        starttime = time.time()
        num = 0
        while num < inf.detect_time:
            if time.time() > starttime + 5:
                return None
            d = self.get_distance()
            if d == None:
                timecosts.clear()
                num = 0
                starttime = time.time()
            elif d < inf.ball_distance:
                num += 1
                timecosts.append(time.time() - starttime)
                starttime = time.time()
            time.sleep(0.06)
        timecosts.sort()
        return timecosts[int(inf.detect_time / 2)]

    # 判断摄像头下是否有球
    def check_ball(self):
        starttime = time.time()
        while time.time() < starttime + 5:
            d = self.get_distance()
            if d < inf.ball_distance:
                return time.time()
        return None


# ####################### 机器视觉类 ######################## #
class Machine_vision():
    def __init__(self, communicate, measurer, w, h):
        self.communicator = communicate
        self.measurer = measurer
        # 搭建opencv环境
        self.topcap = cv2.VideoCapture(inf.top_camera_id)
        if inf.debug_single_camera:
            self.sidecap = self.topcap
        else:
            self.sidecap = cv2.VideoCapture(inf.side_camera_id)
        # 缩小画面，加快运行速度，width、hight为画面宽高，stepth为循环时的步长
        width = w
        height = h
        self.stepth = 10
        self.sidecap.set(5, 15)
        self.sidecap.set(3, width)
        self.sidecap.set(4, height)
        self.topcap.set(5, 15)
        self.topcap.set(3, width)
        self.topcap.set(4, height)
        # 获取基础属性
        ret, self.frame = self.sidecap.read()
        self.sideheight = self.frame.shape[0]
        self.sidewidth = self.frame.shape[1]
        print('侧边摄像头画面属性:长', self.sidewidth, ' 宽', self.sideheight, ' 帧率', self.sidecap.get(5))
        if not inf.debug_single_camera:
            self.sidecap.release()
        ret, self.frame = self.topcap.read()
        self.topheight = self.frame.shape[0]
        self.topwidth = self.frame.shape[1]
        print('顶部摄像头画面属性:长', self.topwidth, ' 宽', self.topheight, ' 帧率', self.sidecap.get(5))
        self.topcap.release()

    # ----------------------------------------------------------- #
    # ----------------------- 第一层：基础层 ----------------------- #
    # ----------------------------------------------------------- #
    # 打开顶部摄像头
    def open_top_camera(self):
        self.topcap = cv2.VideoCapture(inf.top_camera_id)
        self.topcap.set(5, 15)
        self.topcap.set(3, self.topwidth)
        self.topcap.set(4, self.topheight)

    # 打开侧边摄像头
    def open_side_camera(self):
        self.sidecap = cv2.VideoCapture(inf.side_camera_id)
        self.sidecap.set(5, 15)
        self.sidecap.set(3, self.sidewidth)
        self.sidecap.set(4, self.sideheight)

    # 延时实时读取摄像头画面
    def getframe(self, cap, refreshtime=None):
        i = 0
        if refreshtime == None:
            refreshtime = inf.img_read_delay
        while i < refreshtime:
            ret, frame = cap.read()
            i += 1
            cv2.waitKey(1)
        return ret, frame

    # 窗口归一化排列
    def placewin(self, titles):
        num = len(titles)
        places = [[0, 0], [0, 520], [1000, 0], [1000, 520]]
        for i in range(num):
            cv2.moveWindow(titles[i], places[i][0], places[i][1])

    # 统计单色图中某颜色的数量
    def countcolor(self, img):
        return np.sum(img / 255)

    # 颜色过滤
    def getcolor(self, img, color):
        img = np.array(img, np.float32)
        mask = mask1 = mask2 = mask3 = None
        if color == 'r':
            # rg,rb比大于阈值且亮度高于阈值
            bk = 1.7  # rb比
            gk = 1.8  # rg比
            mask1 = img[:, :, 0] * bk - img[:, :, 2]
            mask2 = img[:, :, 1] * gk - img[:, :, 2]
            mask3 = np.where(img[:, :, 2] > 50, 255, 0)
        if color == 'g':
            # rg,gb比大于阈值且亮度高于阈值
            bk = 1.7  # rb比
            gk = 1.8  # rg比
            mask1 = img[:, :, 0] * bk - img[:, :, 1]
            mask2 = img[:, :, 2] * gk - img[:, :, 1]
            mask3 = np.where(img[:, :, 1] > 50, 255, 0)
        elif color == 'b':
            # br,bg比大于阈值且亮度高于阈值
            rk = 1.9  # rb比
            gk = 1.2  # gb比
            mask1 = img[:, :, 2] * rk - img[:, :, 0]
            mask2 = img[:, :, 1] * gk - img[:, :, 0]
            mask3 = np.where(img[:, :, 0] > 50, 255, 0)
        elif color == 'y':
            # rg差值小于阈值,rb比大于阈值且亮度高于阈值
            rg = 0.31  # rg差值比
            bk = 0.26  # rb比
            mask1 = img[:, :, 1] - (1 + rg) * img[:, :, 2]
            mask2 = (1 - rg) * img[:, :, 2] - img[:, :, 1]
            mask3 = img[:, :, 2] * bk - img[:, :, 0]
            mask3 = np.where(mask3 > 0, 255, 0)
            mask4 = np.where(img[:, :, 2] > 50, 255, 0)
            mask3 = cv2.bitwise_and(mask3, mask4)
        elif color == 'w':
            # rg差值小于阈值,gb差值小于阈值且亮度高于阈值
            rg = 0.2  # rb比
            gb = 0.2  # gb比
            mask1 = img[:, :, 1] - (1 + rg) * img[:, :, 2]
            mask2 = (1 - rg) * img[:, :, 2] - img[:, :, 1]
            mask1 = np.where(mask1 < 0, 255, 0)
            mask2 = np.where(mask2 < 0, 255, 0)
            mask3 = cv2.bitwise_and(mask1, mask2)
            mask1 = img[:, :, 0] - (1 + gb) * img[:, :, 1]
            mask2 = (1 - gb) * img[:, :, 1] - img[:, :, 0]
            mask4 = np.where(img[:, :, 2] > 50, 255, 0)
            mask3 = cv2.bitwise_and(mask3, mask4)

        mask1 = np.where(mask1 < 0, 255, 0)
        mask2 = np.where(mask2 < 0, 255, 0)
        mask = cv2.bitwise_and(mask1, mask2)
        mask = cv2.bitwise_and(mask, mask3)
        mask = np.array(mask, np.uint8)
        return mask

    # 阈值滤波
    def imgthreshold(self, img, core, th):
        img = cv2.bilateralFilter(img, core, 300, 50)
        ret, img = cv2.threshold(img, th, 255, cv2.THRESH_TOZERO)
        return img

    # ----------------------------------------------------------- #
    # ----------------------- 第二层：封装层 ----------------------- #
    # ----------------------------------------------------------- #
    # 展示DEBUG窗体
    def show_debug_win(self, read, catch_flag, cirnum, cir, c1, c2=None, block=False):
        if not inf.debug:
            return
        if catch_flag:
            read = cv2.rectangle(read, (int(inf.tar40_x - inf.tar40_delta_x), int(inf.tar40_y - inf.tar40_delta_y)),
                                 (int(inf.tar40_x + inf.tar40_delta_x), int(inf.tar40_y + inf.tar40_delta_y)),
                                 (0, 255, 0), 2)
        else:
            read = cv2.rectangle(read, (int(inf.img_w / 2 - inf.tar40_delta_x), 0),
                                 (int(inf.img_w / 2 + inf.tar40_delta_x), vision.sideheight),
                                 (0, 255, 0), 2)

        if not cirnum == 0:
            if not block:
                cv2.circle(read, (cir[0], cir[1] - cir[2]), cir[2], (255, 255, 255), 3)
            else:
                t = cir[0] - int(cir[3] / 2)
                if t < 0:
                    t = 0
                read = cv2.rectangle(read, (cir[1], t), (cir[1] + cir[4], cir[0] + int(cir[3] / 2)), (0, 255, 255), 2)

        # 界面交互
        def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                print("r ", read[y, x, 2], " ", "g ", read[y, x, 1], " ", "b ", read[y, x, 0])

        cv2.imshow('end-img', read)
        cv2.setMouseCallback("end-img", on_EVENT_LBUTTONDOWN)
        winname = ['end-img', 'one-color-img-' + c1]
        if c2 != None:
            winname.append('one-color-img-' + c2)
        self.placewin(winname)

    # 霍夫圆变换
    def findcircle(self, ori_img, onec_color_img, hf_dp, hf_mindist, hf_param1, hf_param2, hf_minradius, hf_maxredius,
                   hf_maxnum):
        circles = cv2.HoughCircles(onec_color_img, cv2.HOUGH_GRADIENT, dp=hf_dp,
                                   minDist=hf_mindist, param1=hf_param1, param2=hf_param2,
                                   minRadius=hf_minradius, maxRadius=hf_maxredius)
        num = 0
        cir = []
        if circles is not None:
            for i in circles[0]:
                num = num + 1
                i[2] = i[2] + 7 / 3
                print(i[2], round(i[2]))
                cir.append([i[0], i[1], int(round(i[2]))])
                if num == hf_maxnum: break
        return num, cir

    # 根据颜色和轮廓找物体
    def findcircle2(self, onec_color_img, threshold_core, min_radius, check_shape):
        if inf.pc:
            contours, hierarchy = cv2.findContours(onec_color_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        else:
            image, contours, hierarchy = cv2.findContours(onec_color_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        num = 0
        cir = []
        for c in contours:
            (x0, y0), radius = cv2.minEnclosingCircle(c)
            radius = int(radius) + int(threshold_core / 2)
            if check_shape and self.check_circle_rectangle(onec_color_img, [x0, y0, radius]) != 'circle':
                continue
            if radius <= min_radius:
                continue
            cir.append([int(x0), int(y0 + radius), radius])
            num += 1
        if num != 0:
            cir.sort(key=itemgetter(0), reverse=False)
        return num, cir

    # 根据颜色和轮廓找积木
    def findblock(self, onec_color_img, threshold_core, min_radius):
        if inf.pc:
            contours, hierarchy = cv2.findContours(onec_color_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        else:
            image, contours, hierarchy = cv2.findContours(onec_color_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        num = 0
        blo = []
        for c in contours:
            (x0, y0), (width, height), theta = cv2.minAreaRect(c)
            width = int(width) + int(threshold_core / 2)
            height = int(height) + int(threshold_core / 2)
            if width <= min_radius or height <= min_radius:
                continue
            blo.append([int(x0), (y0 + height / 2), width / height, int(width), int(height), abs(x0 - self.topwidth)])
            num += 1
        if num != 0:
            blo.sort(key=itemgetter(5))
        return num, blo

    # 判断是球还是积木
    def check_circle_rectangle(self, img, cir):
        if int(cir[0] - cir[2]) < 0 or int(cir[0] + cir[2]) > img.shape[1] or int(cir[1] - cir[2]) < 0 or int(
                cir[1] + cir[2]) > img.shape[0]:
            print('在边界')
            return 'board'
        imgtemp = img[int(cir[1] - cir[2]):int(cir[1] + cir[2]), int(cir[0] - cir[2]):int(cir[0] + cir[2])]

        nummax = self.countcolor(imgtemp)
        if nummax < 100:
            return 'none'
        if inf.debug:
            cv2.imshow('imgtemp', imgtemp)
        t = int((2 ** 0.5) / 2)
        nummin = self.countcolor(
            img[int(cir[1] - t * cir[2]):int(cir[1] + t * cir[2]), int(cir[0] - t * cir[2]):int(cir[0] + t * cir[2])])
        num = nummax - nummin
        print(num, 2 * (cir[2] ** 2))
        if num > 2 * (cir[2] ** 2) * inf.ball_block_threshold:
            print('球')
            return 'circle'
        else:
            print('矩形')
            return 'rectangle'

    # 正在抓球时树莓派应该进行的操作
    def while_fetching(self, color, deftime, circle_flag=False):
        get_mess_flag = False
        success_flag = False
        # 如果收到抓取成功信息
        if self.communicator != None and self.communicator.listener_getball() == True:
            get_mess_flag = True
            print('抓球结束，判断是否成功')
            # 如果确认抓到了球
            if self.check_fetch(color) == True:
                communicator.writer(inf.success_flag)
                success_flag = True
                return success_flag, get_mess_flag, deftime
            else:
                deftime += 1
                # 如果多次没抓到
                if not circle_flag and deftime >= inf.max_deftime:
                    for i in range(inf.def_movetime):
                        communicator.writer(inf.movedown_quick_flag)
                        cv2.waitKey(inf.img_delay)
                    deftime = 0
                else:
                    cv2.waitKey(inf.img_delay)
                    ret, read = self.getframe(self.topcap)
                communicator.writer(inf.defect_flag)
                time.sleep(2 * inf.catch_delay)
        return success_flag, get_mess_flag, deftime

    # 获取单色图
    def get_one_color_img(self, read, color):
        img = self.getcolor(read, color)
        # 参数
        thresholdcore = 0
        if inf.need_threshold:
            thresholdcore = inf.threshold_core
            img = self.imgthreshold(img, inf.threshold_core, inf.threshild_th)
        if inf.debug:
            cv2.imshow('one-color-img-' + color, img)
        return thresholdcore, img

    # 确定是否抓取成功
    def check_fetch(self, color):
        ret, img = self.getframe(self.topcap)
        img = self.getcolor(img, color)
        num = self.countcolor(img)
        if num > inf.check_fetch_threshold:
            print('抓到了', color, num)
            return True
        print('没抓到')
        return False

    # 走到最佳抓取位置
    def get_close_to_target(self, cir, movetime, ball_flag, place_flag, block_long_flag=False):
        if movetime > inf.max_movetime:
            if block_long_flag:
                communicator.writer(inf.fetch_long_flag)
            else:
                communicator.writer(inf.fetch_short_flag)
            return True
        if place_flag != 3:
            if (place_flag == 1 and inf.ourcolor == 'b') or (place_flag == 2 and inf.ourcolor == 'r'):
                if cir[0] < inf.tar40_x - inf.tar40_delta_x:
                    communicator.writer(inf.moveup_flag)
                elif cir[0] > inf.tar40_x + inf.tar40_delta_x:
                    communicator.writer(inf.movedown_flag)
                elif cir[1] < inf.tar40_y - inf.tar40_delta_y:
                    communicator.writer(inf.moveright_flag)
                elif cir[1] > inf.tar40_y + inf.tar40_delta_y:
                    communicator.writer(inf.moveleft_flag)
                else:
                    if ball_flag:
                        communicator.writer(inf.fetch_ball_flag)
                    elif block_long_flag:
                        communicator.writer(inf.fetch_long_flag)
                    else:
                        communicator.writer(inf.fetch_short_flag)
                    return True
            else:
                if cir[0] < inf.tar40_x - inf.tar40_delta_x:
                    communicator.writer(inf.movedown_flag)
                elif cir[0] > inf.tar40_x + inf.tar40_delta_x:
                    communicator.writer(inf.moveup_flag)
                elif cir[1] < inf.tar40_y - inf.tar40_delta_y:
                    communicator.writer(inf.moveleft_flag)
                elif cir[1] > inf.tar40_y + inf.tar40_delta_y:
                    communicator.writer(inf.moveright_flag)
                else:
                    if ball_flag:
                        communicator.writer(inf.fetch_ball_flag)
                    elif block_long_flag:
                        communicator.writer(inf.fetch_long_flag)
                    else:
                        communicator.writer(inf.fetch_short_flag)
                    return True
        else:
            if cir[0] < inf.tar40_x - inf.tar40_delta_x:
                communicator.writer(inf.moveleft_flag)
            elif cir[0] > inf.tar40_x + inf.tar40_delta_x:
                communicator.writer(inf.moveright_flag)
            elif cir[1] < inf.tar40_y - inf.tar40_delta_y:
                communicator.writer(inf.movedown_flag)
            elif cir[1] > inf.tar40_y + inf.tar40_delta_y:
                communicator.writer(inf.moveup_flag)
            else:
                communicator.writer(inf.fetch_ball_flag)
                return True
        return False

    # 判断二维码
    def check_code(self, img):
        if inf.debug:
            cv2.imshow('code', img)
        asc = {'r': 'R', 'b': 'B'}
        barcodes = pyzbar.decode(img)
        if len(barcodes) == 0:
            return None
        print('检测到二维码')
        for barcode in barcodes:
            # 提取二维码的边界框的位置
            # 画出图像中条形码的边界框
            (x, y, w, h) = barcode.rect
            # 提取二维码数据为字节对象，所以如果我们想在输出图像上
            # 画出来，就需要先将它转换成字符串
            barcodeData = barcode.data.decode("utf-8")
            barcodeType = barcode.type
            # 向终端打印条形码数据和条形码类型
            print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
            if barcodeData == asc[inf.ourcolor]:
                return True
        return False

    # ------------------------------------------------------- #
    # --------------------- 第三层：应用层 --------------------- #
    # ------------------------------------------------------- #

    # begin=False时判断圆盘上是否存在己方球,begin=True时判断起始点颜色
    def checkcolor(self, begin=False):
        foundtimes = 0
        alltimes = 0
        if not begin:
            while alltimes < 80:
                alltimes += 1
                ret, read = self.topcap.read()
                img = self.getcolor(read, inf.ourcolor)
                num = self.countcolor(img)
                # 通讯
                if num != 0:
                    foundtimes += 1
                    if foundtimes > 8:
                        self.communicator.writer(self.ourcolor)
                        print('存在红蓝球')
                        return self.ourcolor
                if inf.debug:
                    cv2.imshow('ori-img', read)
                    cv2.imshow('one-color-img', img)

                    # 界面交互
                    def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
                        if event == cv2.EVENT_LBUTTONDOWN:
                            print("r ", read[y, x, 2], " ", "g ", read[y, x, 1], " ", "b ", read[y, x, 0])

                    cv2.setMouseCallback("ori-img", on_EVENT_LBUTTONDOWN)
                    self.placewin(['ori-img', 'one-color-img'])
                cv2.waitKey(10)
                if self.communicator != None and self.communicator.listener_end() == True:
                    print('单片机叫停，返回监听状态')
                    return 'y'
        else:
            ret, read = self.getframe(self.topcap)
            imgr = self.getcolor(read, 'r')
            imgb = self.getcolor(read, 'b')
            rnum = self.countcolor(imgr)
            bnum = self.countcolor(imgb)
            print(rnum, bnum)
            print(self.topwidth, self.topheight)
            if inf.debug:
                cv2.imshow('read', read)
                cv2.waitKey(1)
            if rnum > self.topwidth * self.topheight * 0.01 or bnum > self.topwidth * self.topheight * 0.01:
                if rnum > bnum:
                    self.ourcolor = inf.ourcolor = 'r'
                    self.communicator.writer(inf.we_are_red_flag)
                else:
                    self.ourcolor = inf.ourcolor = 'b'
                    self.communicator.writer(inf.we_are_blue_flag)
                return self.ourcolor
        print('未发现红蓝球')
        self.communicator.writer('y')
        return 'y'

    # 抓取立柱球，有黄色，无积木，抓所有球
    def get_pole_ball(self):
        getnum=0
        fetching = False
        deftime = 0
        nowcolor = None
        movetime = 0
        # 如果当前正在抓取
        while :
            # 如果当前正在抓取
            if fetching:
                success, get_mess_flag, deftime = self.while_fetching(nowcolor, deftime)
                if not get_mess_flag or success:
                    continue
                else:
                    fetching = False
                    movetime = 0
            # 读取画面
            ret, read = self.getframe(self.topcap)
            thresholdcore, imgour = self.get_one_color_img(read, inf.ourcolor)
            thresholdcore, imgy = self.get_one_color_img(read, 'y')
            imgg = self.getcolor(read, 'g')
            numg = self.countcolor(imgg)
            if numg < 1000:
                communicator.writer(inf.unfound_flag)
            # 找圆
            if inf.use_hough:
                numour, cirour = self.findcircle(read, imgour, inf.hough_dp, 30, 300, inf.hough_ballsh, 1, 150, 1)
                numy, ciry = self.findcircle(read, imgy, inf.hough_dp, 30, 300, inf.hough_ballsh, 1, 150, 1)
            else:
                numour, cirour = self.findcircle2(imgour, thresholdcore, inf.ball_minradius, False)
                numy, ciry = self.findcircle2(imgy, thresholdcore, inf.ball_minradius, False)
            # 通讯
            if numour + numy == 0:
                communicator.writer(inf.moveup_flag)
                movetime = 0
                cir = None
            else:
                # 选择最近的球进行抓取
                if numour == 0:
                    cir = ciry[0]
                    nowcolor = 'y'
                elif numy == 0:
                    cir = cirour[0]
                    nowcolor = inf.ourcolor
                elif cirour[0][2] > ciry[0][2]:
                    cir = cirour[0]
                    nowcolor = inf.ourcolor
                else:
                    cir = ciry[0]
                    nowcolor = 'y'
                fetching = self.get_close_to_target(cir, movetime, True, 1)
            self.show_debug_win(read, True, numour + numy, cir, inf.ourcolor, 'y')

            if self.communicator != None and self.communicator.listener_end() == True:
                print('单片机叫停，返回监听状态')
                return None
            cv2.waitKey(inf.img_delay)
        return nowcolor

    # 获取平台球，有黄色，有积木
    def get_platform_ball(self):
        success = False
        fetching = False
        deftime = 0
        nowcolor = None
        movetime = 0
        while not success:
            # 如果当前正在抓取
            if fetching:
                success, get_mess_flag, deftime = self.while_fetching(nowcolor, deftime)
                if not get_mess_flag or success:
                    continue
                else:
                    fetching = False
                    movetime = 0
            # 读取画面
            ret, read = self.getframe(self.topcap)
            thresholdcore, imgour = self.get_one_color_img(read, inf.ourcolor)
            thresholdcore, imgy = self.get_one_color_img(read, 'y')
            imgg = self.getcolor(read, 'g')
            numg = self.countcolor(imgg)
            if numg < 1000:
                communicator.writer(inf.unfound_flag)
            # 找圆
            if inf.use_hough:
                numour, cirour = self.findcircle(read, imgour, inf.hough_dp, 30, 300, inf.hough_ballsh, 1, 150, 1)
                numy, ciry = self.findcircle(read, imgy, inf.hough_dp, 30, 300, inf.hough_ballsh, 1, 150, 1)
            else:
                numour, cirour = self.findcircle2(imgour, thresholdcore, inf.ball_minradius, not inf.block_usecode)
                numy, ciry = self.findcircle2(imgy, thresholdcore, inf.ball_minradius, False)
            # 通讯
            if numour + numy == 0:
                communicator.writer(inf.movedown_quick_flag)
                movetime = 0
                cir = None
            else:
                # 选择最近的球进行抓取
                if numour == 0:
                    cir = ciry[0]
                    nowcolor = 'y'
                elif numy == 0:
                    cir = cirour[0]
                    nowcolor = inf.ourcolor
                elif cirour[0][2] > ciry[0][2]:
                    cir = cirour[0]
                    nowcolor = inf.ourcolor
                else:
                    cir = ciry[0]
                    nowcolor = 'y'
                fetching = self.get_close_to_target(cir, movetime, True, 2)
            self.show_debug_win(read, True, numour + numy, cir, inf.ourcolor, 'y')
            if self.communicator != None and self.communicator.listener_end() == True:
                print('单片机叫停，返回监听状态')
                return None
            cv2.waitKey(inf.img_delay)
        return nowcolor

    # 抓取圆盘球，有黄色，无积木,尽量抓己方球
    def get_circle_ball_unmovable(self, color):
        success = False
        fetching = False
        deftime = 0
        movetime = 0
        while not success:
            # 如果当前正在抓取
            if fetching:
                success, get_mess_flag, deftime = self.while_fetching(color, deftime)
                # print(success,get_mess_flag)
                if not get_mess_flag or success:
                    continue
                else:
                    fetching = False
                    movetime = 0
            # 读取画面
            ret, read = self.getframe(self.topcap)
            thresholdcore, img = self.get_one_color_img(read, color)
            imgg = self.getcolor(read, 'g')
            numg = self.countcolor(imgg)
            if numg < 1000:
                communicator.writer(inf.unfound_flag)
            # 找圆
            if inf.use_hough:
                num, cir = self.findcircle(read, img, inf.hough_dp, 30, 300, inf.hough_ballsh, 1, 150, 1)
            else:
                num, cir = self.findcircle2(img, thresholdcore, inf.ball_minradius, False)
            # 通讯
            if num == 0:
                if inf.ourcolor == 'b':
                    communicator.writer(inf.moveright_flag)
                else:
                    communicator.writer(inf.moveleft_flag)
                movetime = 0
                cir = [None]
            else:
                fetching = self.get_close_to_target(cir[0], movetime, True, 3)
            self.show_debug_win(read, True, num, cir[0], color)
            if self.communicator != None and self.communicator.listener_end() == True:
                print('单片机叫停，返回监听状态')
                return None
            cv2.waitKey(inf.img_delay)
        return color

    # 抓取动态圆盘球
    def get_circle_ball_movable(self, color):
        success = False
        fetching = False
        deftime = 0
        # 获取圆盘球时间间隙
        while inf.circle_time == None:
            inf.circle_time = self.measurer.get_aver_time()
            print('时间间隔为：', inf.circle_time)
        while not success:
            # 如果当前正在抓取
            if fetching:
                success, get_mess_flag, deftime = self.while_fetching(color, deftime, True)
                # print(success,get_mess_flag)
                if not get_mess_flag or success:
                    continue
                else:
                    fetching = False
            # 获取某次有球的时刻
            ball_time = self.measurer.check_ball()
            while ball_time == None:
                ball_time = self.measurer.check_ball()
                print('未检测到球')
            print('检测到球了')
            # 预估抓取时刻
            if inf.debug:
                if inf.circle_time * inf.camera_arm_angle < inf.catch_delay:
                    print('warning:爪子运动比球运动慢')
                print('球运动时间:', inf.circle_time * inf.camera_arm_angle, ' 爪子运动时间:', inf.catch_delay)
            catch_time = ball_time + inf.circle_time * inf.camera_arm_angle - inf.catch_delay
            # 读取画面
            ret, read = self.getframe(self.topcap, 5)
            thresholdcore, img = self.get_one_color_img(read, color)
            # 判断颜色
            num = self.countcolor(img)
            # 是目标则抓取否则等待下次
            if num > inf.check_fetch_threshold:
                print('准备抓球')
                while time.time() <= catch_time:
                    pass
                self.communicator.writer(inf.fetch_ball_flag)
                fetching = True
                if inf.debug:
                    print('实际抓取与预计差值:', time.time() - catch_time)
                    if time.time() - catch_time > 0.15:
                        print('warning:检测速度比球运动慢')
            cv2.imshow('img', read)
            if self.communicator != None and self.communicator.listener_end() == True:
                print('单片机叫停，返回监听状态')
                return None
            cv2.waitKey(1)
        return color

    # 标定放球口
    def get_goal(self, color):
        turnflag = False
        if color == 'r' or color == 'b':
            turnflag = True
        while 1:
            ret, read = self.getframe(self.sidecap)
            thresholdcore, img = self.get_one_color_img(read, 'y')
            # 找圆
            if inf.use_hough:
                num, cir = self.findcircle(read, img, inf.hough_dp, 30, 300, inf.hough_ballsh, 1, 150, 1)
            else:
                num, cir = self.findcircle2(img, thresholdcore, inf.ball_minradius + 20, False)
            # 通讯
            if num == 0:
                communicator.writer(inf.movedown_quick_flag)
            else:
                cir = cir[0]
                if (cir[0] < inf.img_w / 2 - inf.tar40_delta_x and inf.ourcolor == 'b') or (
                        cir[0] > inf.img_w / 2 + inf.tar40_delta_x and inf.ourcolor == 'r'):
                    communicator.writer(inf.moveup_flag)
                elif (cir[0] > inf.img_w / 2 + inf.tar40_delta_x and inf.ourcolor == 'b') or (
                        cir[0] < inf.img_w / 2 - inf.tar40_delta_x and inf.ourcolor == 'r'):
                    communicator.writer(inf.movedown_flag)
                else:
                    if turnflag:
                        communicator.writer(inf.turn_bingo_flag)
                    else:
                        communicator.writer(inf.bingo_flag)
                    return
            self.show_debug_win(read, False, num, cir, color)
            if self.communicator != None and self.communicator.listener_end() == True:
                print('单片机叫停，返回监听状态')
                return True
            cv2.waitKey(inf.img_delay)

    # 抓取积木
    def get_block_usecode(self):
        success = False
        fetching = False
        deftime = 0
        movetime = 0
        while not success:
            # 如果当前正在抓取
            if fetching:
                success, get_mess_flag, deftime = self.while_fetching('w', deftime)
                if not get_mess_flag or success:
                    continue
                else:
                    fetching = False
                    movetime = 0
            # 读取画面
            ret, read = self.getframe(self.topcap)
            thresholdcore, img = self.get_one_color_img(read, 'w')
            imgg = self.getcolor(read, 'g')
            numg = self.countcolor(imgg)
            if numg < 1000:
                communicator.writer(inf.unfound_flag)
            # 找积木
            numour, blo = self.findblock(img, thresholdcore, inf.ball_minradius - 2)
            # 通讯
            if numour == 0:
                communicator.writer(inf.movedown_quick_flag)
                movetime = 0
                blo = None
            else:
                blo = blo[0]
                # [int(x0), int(y0 - height / 2), width / height, int(width), int(height), abs(x0 - self.topwidth)
                t = blo[0] - int(blo[3] / 2)
                if t < 0:
                    t = 0
                result = self.check_code(read[blo[1]:blo[1] + blo[4], t:blo[0] + int(blo[3] / 2)])
                if result == None:
                    movetime += 0.3
                    continue
                elif result == False:
                    for i in range(inf.def_movetime):
                        communicator.writer(inf.movedown_quick_flag)
                        cv2.waitKey(inf.img_delay)
                    deftime = 0
                    movetime = 0
                    continue
                long_flag = False
                if blo[2] > inf.block_long_short_threshold:
                    long_flag = True
                fetching = self.get_close_to_target(blo, movetime, False, 2, long_flag)
            self.show_debug_win(read, True, numour, blo, inf.ourcolor, block=True)
            if self.communicator != None and self.communicator.listener_end() == True:
                print('单片机叫停，返回监听状态')
                return None
            cv2.waitKey(inf.img_delay)
        return inf.ourcolor

    # 抓取积木
    def get_block_usecolor(self):
        success = False
        fetching = False
        deftime = 0
        movetime = 0
        while not success:
            # 如果当前正在抓取
            if fetching:
                success, get_mess_flag, deftime = self.while_fetching(inf.ourcolor, deftime)
                if not get_mess_flag or success:
                    continue
                else:
                    fetching = False
                    movetime = 0
            # 读取画面
            ret, read = self.getframe(self.topcap)
            thresholdcore, imgour = self.get_one_color_img(read, inf.ourcolor)
            imgg = self.getcolor(read, 'g')
            numg = self.countcolor(imgg)
            if numg < 1000:
                communicator.writer(inf.unfound_flag)
            # 找积木
            numour, blo = self.findblock(imgour, thresholdcore, inf.ball_minradius - 2)
            # 通讯
            if numour == 0:
                communicator.writer(inf.movedown_quick_flag)
                movetime = 0
                blo = None
            else:
                blo = blo[0]
                long_flag = False
                if blo[2] > inf.block_long_short_threshold:
                    long_flag = True
                fetching = self.get_close_to_target(blo, movetime, False, 2, long_flag)
            self.show_debug_win(read, True, numour, blo, inf.ourcolor, block=True)
            if self.communicator != None and self.communicator.listener_end() == True:
                print('单片机叫停，返回监听状态')
                return None
            cv2.waitKey(inf.img_delay)
        return inf.ourcolor


def test():
    # 测试代码
    # vision.getball('r', inf.ball_movable, inf.debug)
    # vision.checkblock(inf.block_usecode)
    inf.ourcolor = inf.next_ball_color = 'b'
    vision.open_top_camera()
    if inf.block_usecode:
        inf.now_ball_color = vision.get_block_usecode()
    else:
        inf.now_ball_color = vision.get_block_usecolor()
    vision.topcap.release()


# ####################### 入口函数 ######################## #
if __name__ == "__main__":
    # 搭建基础对象
    inf = Information()
    communicator = Communicator(inf.serial_port, inf.serial_rate, inf.serial_timeout)
    measurer = Measurer()
    vision = Machine_vision(communicator, measurer, inf.img_w, inf.img_h)
    if inf.debug:
        test()
        pass

    # 状态关键字说明:
    # -1-错误状态，0-监听状态，1-判断己方颜色，2-判断圆盘球，3-取圆盘球，4-取立柱球，5-取条形平台球，6-取条形平台积木，7-放球
    state = 0
    # 监听信号
    stopflag = 0
    while not stopflag:
        state = communicator.listener_begin()
        if state == 0:
            # 结束监听
            stopflag = True
        elif state == 1:
            # 判断己方颜色
            vision.open_top_camera()
            inf.next_ball_color = vision.checkcolor(begin=True)
            vision.topcap.release()
        elif state == 2:
            # 判断圆盘球数
            vision.open_top_camera()
            inf.next_ball_color = vision.checkcolor(begin=False)
            vision.topcap.release()
        elif state == 3:
            # 取圆盘球
            if inf.ball_movable:
                vision.open_side_camera()
                inf.now_ball_color = vision.get_circle_ball_movable(inf.next_ball_color)
                vision.sidecap.release()
            else:
                vision.open_top_camera()
                inf.now_ball_color = vision.get_circle_ball_unmovable(inf.next_ball_color)
                vision.topcap.release()

        elif state == 4:
            # 取立柱球
            vision.open_top_camera()
            inf.now_ball_color = vision.get_pole_ball()
            vision.topcap.release()
        elif state == 5:
            # 取条形平台球,如果不是初赛，要考虑己方和黄色两种颜色
            vision.open_top_camera()
            if inf.ball_movable:
                inf.now_ball_color = vision.get_platform_ball()
            else:
                inf.now_ball_color = vision.get_circle_ball_unmovable(inf.ourcolor)
            vision.topcap.release()
        elif state == 6:
            # 取条形平台积木
            vision.open_top_camera()
            if inf.block_usecode:
                inf.now_ball_color = vision.get_block_usecode()
            else:
                inf.now_ball_color = vision.get_block_usecolor()
            vision.topcap.release()
        elif state == 7:
            # 放球
            vision.open_side_camera()
            vision.get_goal(inf.now_ball_color)
            vision.sidecap.release()
        elif state == -1:
            print('程序执行出错')
        state = 0
    print('程序执行完毕')
