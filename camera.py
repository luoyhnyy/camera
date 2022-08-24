import configparser
import math
import re
import random
import shutil
import socket
import threading
import time
import tkinter.ttk as tk
from tkinter import *
from tkinter.filedialog import *
from tkinter.messagebox import *
import PIL
import cv2
import numpy as np
import pandas as pd
import windnd
from PIL import Image, ImageTk
from pyueye import ueye
from scipy.interpolate import interp1d
from scipy.interpolate import lagrange
from scipy.interpolate import griddata
from scipy.interpolate import Rbf
import itertools
from ImageConvert import *
from MVSDK import *
import gc
from xml.dom.minidom import parseString
import datetime

"""
相机网格服务器 作者：华强方特(深圳)智能技术有限公司 AVCL系统工程中心-罗尧 2021年10月
"""
# 关于
def explain():
    showinfo("关于", "华强方特（深圳）智能技术有限公司\nAVCL系统工程中心\t2021年10月")
# 创建文件夹
def mkdir(path):
    if not os.path.exists(path):  # 判断是否存在文件夹如果不存在则创建文件夹
        os.makedirs(path)
    return path
# 删除文件夹
def dedir(path):
    if os.path.exists(path):  # 判断是否存在文件夹如果存在则删除文件夹
        shutil.rmtree(path)
    return path
# 删除文件
def defile(path):
    if os.path.exists(path):  # 判断是否存在文件如果存在则删除文件
        os.remove(path)
    return path
# 清空文件夹
def defiles(path):
    filelist = os.listdir(path)
    for f in filelist:
        filepath = os.path.join(path,f)
        os.remove(filepath)
# 复制文件
def copyfi(path1,path2):
    if os.path.exists(path1):  # 判断是否存在文件如果存在则复制文件
        shutil.copy(path1,path2)
    return path2
# 获取绝对路径
def resource_path(path):
    base_path = getattr(sys,'_MEIPASS',os.path.dirname(os.path.abspath(__file__)))
    # print(os.path.join(base_path,path))
    return os.path.join(base_path,path)
paths = os.path.abspath('.') + r'\DATA'
log_paths = os.path.abspath('.') + r'\DATA\LOG'
pic_path = paths[:]
mkdir(paths)
mkdir(os.path.join(os.path.abspath('.'),r'DATA\json'))
mkdir(os.path.join(os.path.abspath('.'),r'DATA\uv'))
mkdir(os.path.join(os.path.abspath('.'),r'DATA\xyz'))
mkdir(log_paths)
log_path = log_paths + r"\%s.log"%(datetime.date.today().strftime("%Y%m%d"))
log = open(log_path, 'a')
log.write(str(datetime.datetime.now()) + "  启动网格服务器..." + '\n')
log.close()
# 读取图像
def readimg(path,flags=-1): #flags=-1是原格式，0是单通道，1是3通道
    img = cv2.imdecode(np.fromfile(path, dtype=np.uint8), flags)
    return img
# 保存图像
def saveimg(img,path):
    cv2.imencode('.png',img)[1].tofile(path)
# 计算平方和开根号
def segment(a,b):
    return math.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)
# 设置华睿相机增益
def setGain(camera, dVal):
    gainRawNode = pointer(GENICAM_DoubleNode())
    gainRawNodeInfo = GENICAM_DoubleNodeInfo()
    gainRawNodeInfo.pCamera = pointer(camera)
    gainRawNodeInfo.attrName = b"GainRaw"
    nRet = GENICAM_createDoubleNode(byref(gainRawNodeInfo), byref(gainRawNode))
    if nRet != 0:
        showtext("create gainRaw Node fail!", 'red')
        return -1
    nRet = gainRawNode.contents.setValue(gainRawNode, c_double(dVal))
    if nRet != 0:
        showtext("set gainRaw value [%f]us fail!"  % (dVal), 'red')
        gainRawNode.contents.release(gainRawNode) # 释放相关资源
        return -1
    gainRawNode.contents.release(gainRawNode) # 释放节点资源
    return 0
# 设置华睿相机曝光时间
def setExposureTime(camera, dVal):
    exposureTimeNode = pointer(GENICAM_DoubleNode())
    exposureTimeNodeInfo = GENICAM_DoubleNodeInfo()
    exposureTimeNodeInfo.pCamera = pointer(camera)
    exposureTimeNodeInfo.attrName = b"ExposureTime"
    nRet = GENICAM_createDoubleNode(byref(exposureTimeNodeInfo), byref(exposureTimeNode))
    if nRet != 0:
        showtext("create ExposureTime Node fail!", 'red')
        return -1
    nRet = exposureTimeNode.contents.setValue(exposureTimeNode, c_double(dVal))
    if nRet != 0:
        showtext("set ExposureTime value [%f]us fail!"  % (dVal), 'red')
        exposureTimeNode.contents.release(exposureTimeNode) # 释放相关资源
        return -1
    exposureTimeNode.contents.release(exposureTimeNode) # 释放节点资源
    return 0
# 更改增益曝光全局变量
def changegp(a):
    global changegainexposure
    changegainexposure = 1
# 连接相机
def concam(camID):
    try:
        if cammodel.get() == "IDS":
            hCam = ueye.HIDS(int(camID))  # 0-254：具有指定摄影机ID的摄影机
            sInfo = ueye.SENSORINFO()  # 传感器信息
            cInfo = ueye.CAMINFO()  # 相机信息
            pcImageMemory = ueye.c_mem_p()
            MemID = ueye.int()
            pitch = ueye.INT()
            rectAOI = ueye.IS_RECT()
            nBitsPerPixel = ueye.INT(24) # 24:颜色模式为每像素位；单色模式为每像素8位
            channels = 3  # 3:颜色模式（RGB）的通道;单色模式采用1个通道
            m_nColorMode = ueye.INT()  # Y8/RGB16/RGB24/REG32
            bytes_per_pixel = int(nBitsPerPixel / 8)
            nRet = ueye.is_InitCamera(hCam, None) # 启动驱动程序并建立与摄像头的连接
            if nRet != ueye.IS_SUCCESS:
                showtext('连接相机%s失败'%(camID), 'red')
                ueye.is_FreeImageMem(hCam, pcImageMemory, MemID)
                ueye.is_ExitCamera(hCam)
                gc.collect()
                return 0,0,0,0,0,0,0,0
            nRet = ueye.is_GetCameraInfo(hCam, cInfo) # 读取非易失性相机存储器中硬编码的数据，并将其写入cInfo指向的数据结构
            if nRet != ueye.IS_SUCCESS:
                showtext('读取相机%s硬编码失败'%(camID), 'red')
                return 0,0,0,0,0,0,0,0
            nRet = ueye.is_GetSensorInfo(hCam, sInfo) # 读取传感器信息
            if nRet != ueye.IS_SUCCESS:
                showtext('读取相机%s传感器失败'%(camID), 'red')
                return 0,0,0,0,0,0,0,0
            nRet = ueye.is_ResetToDefault(hCam) # 重置相机参数
            if nRet != ueye.IS_SUCCESS:
                showtext('重置相机%s参数失败'%(camID), 'red')
                return 0,0,0,0,0,0,0,0
            nRet = ueye.is_SetDisplayMode(hCam, ueye.IS_SET_DM_DIB) # 将显示模式设置为DIB
            m_nColorMode = ueye.IS_CM_MONO8 # 颜色模式
            nBitsPerPixel = ueye.INT(8) # 比特像素
            bytes_per_pixel = int(nBitsPerPixel / 8) # 每像素字节数
            nRet = ueye.is_AOI(hCam, ueye.IS_AOI_IMAGE_GET_AOI, rectAOI, ueye.sizeof(rectAOI))
            if nRet != ueye.IS_SUCCESS:
                showtext('设置相机%sAOI失败' % (camID), 'red')
                return 0,0,0,0,0,0,0,0
            width = rectAOI.s32Width  # 最大图像宽度
            height = rectAOI.s32Height  # 最大图像宽度
            showtext('相机%s型号:%s' % (camID,sInfo.strSensorName.decode('utf-8')), 'black')# 相机型号
            showtext('相机%s序列号:%s' % (camID,cInfo.SerNo.decode('utf-8')), 'black')# 相机序列号
            # 为尺寸由宽度和高度定义的图像分配图像内存，颜色深度由nbitsperpoixel定义
            nRet = ueye.is_AllocImageMem(hCam, width, height, nBitsPerPixel, pcImageMemory, MemID)
            if nRet != ueye.IS_SUCCESS:
                showtext('相机%s分配图像内存失败' % (camID), 'red')
                return 0,0,0,0,0,0,0,0
            else:
                nRet = ueye.is_SetImageMem(hCam, pcImageMemory, MemID) # 使指定的图像内存成为活动内存
                if nRet != ueye.IS_SUCCESS:
                    showtext('相机%s设置活动内存失败' % (camID), 'red')
                    return 0,0,0,0,0,0,0,0
                else:
                    nRet = ueye.is_SetColorMode(hCam, m_nColorMode) # 设置所需的颜色模式
            nRet = ueye.is_CaptureVideo(hCam, ueye.IS_DONT_WAIT) # 激活摄像机的实时视频模式（自由运行模式）
            if nRet != ueye.IS_SUCCESS:
                showtext('相机%s激活相机的实时视频模式失败' % (camID), 'red')
                return 0,0,0,0,0,0,0,0
            nRet = ueye.is_InquireImageMem(hCam, pcImageMemory, MemID, width, height, nBitsPerPixel, pitch) # 启用现有图像内存序列的队列模式
            if nRet != ueye.IS_SUCCESS:
                showtext('相机%s启用队列模式失败' % (camID), 'red')
                return 0,0,0,0,0,0,0,0
            return hCam, MemID, pcImageMemory, width, height, nBitsPerPixel, pitch, bytes_per_pixel
        else:
            g_cameraStatusUserInfo = b"statusInfo"
            def deviceLinkNotify(connectArg, linkInfo):  # 相机连接状态回调函数
                if EVType.offLine == connectArg.contents.m_event:
                    showtext("camera has off line, userInfo [%s]" % (c_char_p(linkInfo).value), 'red')
                elif EVType.onLine == connectArg.contents.m_event:
                    showtext("camera has on line, userInfo [%s]" % (c_char_p(linkInfo).value), 'red')
            connectCallBackFuncEx = connectCallBackEx(deviceLinkNotify)
            system = pointer(GENICAM_System())
            nRet = GENICAM_getSystemInstance(byref(system))
            if nRet != 0:
                showtext("获取系统单例失败!", 'red')
                return 0,0,0,0
            cameraList = pointer(GENICAM_Camera())  # 发现相机
            cameraCnt = c_uint()  # value发现的相机数量
            nRet = system.contents.discovery(system, byref(cameraList), byref(cameraCnt),c_int(GENICAM_EProtocolType.typeAll))
            if nRet != 0 or cameraCnt.value < 1:
                showtext("连接相机%s失败!"%(camID), 'red')
                return 0,0,0,0
            camera = ''
            if camID > len(camseriallist):
                showtext("无相机%s对应序列号!" % (camID), 'red')
                return 0, 0, 0, 0
            for i in range(cameraCnt.value):
                if cameraList[i].getSerialNumber(cameraList[i]).decode() == camseriallist[camID - 1]:
                    camera = cameraList[i] # 相机id
                    break
            if camera == '':
                showtext("无相机%s序列号对应相机!" % (camID), 'red')
                return 0, 0, 0, 0
            # camera = cameraList[camID - 1]  # 相机id
            showtext("相机型号：" + camera.getModelName(camera).decode(), 'black')
            showtext("序列号：" + camera.getSerialNumber(camera).decode(), 'black')
            nRet = camera.connect(camera, c_int(GENICAM_ECameraAccessPermission.accessPermissionControl))
            if nRet != 0:
                showtext("连接相机%s失败!"%(camID), 'red')
                return 0,0,0,0
            eventSubscribe = pointer(GENICAM_EventSubscribe())  # 注册相机连接状态回调
            eventSubscribeInfo = GENICAM_EventSubscribeInfo()
            eventSubscribeInfo.pCamera = pointer(camera)
            nRet = GENICAM_createEventSubscribe(byref(eventSubscribeInfo), byref(eventSubscribe))
            if nRet != 0:
                showtext("创建事件订阅失败!", 'red')
                return 0,0,0,0
            nRet = eventSubscribe.contents.subscribeConnectArgsEx(eventSubscribe, connectCallBackFuncEx,g_cameraStatusUserInfo)
            if nRet != 0:
                showtext("订阅失败!", 'red')
                eventSubscribe.contents.release(eventSubscribe)  # 释放相关资源
                return 0,0,0,0
            eventSubscribe.contents.release(eventSubscribe)  # 不再使用时，需释放相关资源
            if nRet != 0:
                showtext("subscribeCameraStatus fail!", 'red')
                return 0,0,0,0
            streamSourceInfo = GENICAM_StreamSourceInfo()  # 创建流对象
            # streamSourceInfo.channelId = 0
            streamSourceInfo.channelId = camID-1
            streamSourceInfo.pCamera = pointer(camera)
            streamSource = pointer(GENICAM_StreamSource())
            nRet = GENICAM_createStreamSource(pointer(streamSourceInfo), byref(streamSource))
            if nRet != 0:
                showtext("创建流对象失败!", 'red')
                return 0,0,0,0
            trigModeEnumNode = pointer(GENICAM_EnumNode())
            trigModeEnumNodeInfo = GENICAM_EnumNodeInfo()
            trigModeEnumNodeInfo.pCamera = pointer(camera)
            trigModeEnumNodeInfo.attrName = b"TriggerMode"
            nRet = GENICAM_createEnumNode(byref(trigModeEnumNodeInfo), byref(trigModeEnumNode))
            if nRet != 0:
                showtext("create TriggerMode Node fail!", 'red')
                streamSource.contents.release(streamSource)  # 释放相关资源
                return 0,0,0,0
            nRet = trigModeEnumNode.contents.setValueBySymbol(trigModeEnumNode, b"Off")  # 自由拉流：TriggerMode 需为 off
            if nRet != 0:
                showtext("set TriggerMode value [Off] fail!", 'red')
                trigModeEnumNode.contents.release(trigModeEnumNode)  # 释放相关资源
                streamSource.contents.release(streamSource)
                return 0,0,0,0
            trigModeEnumNode.contents.release(trigModeEnumNode)  # 需要释放Node资源
            nRet = streamSource.contents.startGrabbing(streamSource, c_ulonglong(0),c_int(GENICAM_EGrabStrategy.grabStrartegySequential))
            if nRet != 0:
                showtext("开始拉流失败!", 'red')
                streamSource.contents.release(streamSource)  # 释放相关资源
                return 0,0,0,0
            return camera,streamSource,connectCallBackFuncEx,g_cameraStatusUserInfo
    except Exception as e:
        showtext(str(e.__traceback__.tb_lineno)+str(e), 'red')
# 打开/关闭相机
def opencam():
    global mousetype,killcam,photocam,client_send,changegainexposure
    try:
        if mousetype == 0:
            camID1,camID2,camID3 = camID1val.get(),camID2val.get(),camID3val.get()
            changegainexposure = 1
            if cammodel.get() == "IDS":
                camw1, camh1 = 640, 480
                hCam1, MemID1, pcImageMemory1, camwid1, camhei1, nBitsPerPixel1, pitch1, bytes_per_pixel1 = concam(camID1)
                if camhei1 == 0 and camhei1 == 0:
                    return
                if vali.get() == 2:  # 打开两个相机
                    camw2, camh2 = 440, 330
                    hCam2, MemID2, pcImageMemory2, camwid2, camhei2, nBitsPerPixel2, pitch2, bytes_per_pixel2 = concam(camID2)
                    if camhei2 == 0 and camhei2 == 0:
                        return
                elif vali.get() == 3:  # 打开三个相机
                    camw2, camh2, camw3, camh3 = 320, 240, 320, 240
                    hCam2, MemID2, pcImageMemory2, camwid2, camhei2, nBitsPerPixel2, pitch2, bytes_per_pixel2 = concam(camID2)
                    hCam3, MemID3, pcImageMemory3, camwid3, camhei3, nBitsPerPixel3, pitch3, bytes_per_pixel3 = concam(camID3)
                    if (camhei2 == 0 and camhei2 == 0) or (camhei3 == 0 and camhei3 == 0):
                        return
            else:
                camw1, camh1, camw2, camh2, camw3, camh3 = 720, 480, 360, 240, 360, 240
                camera1,stream1,FuncEx1,cameraInfo1 = concam(camID1)
                if camera1 == 0:
                    return
                if vali.get() == 2:  # 打开两个相机
                    camera2, stream2, FuncEx2, cameraInfo2 = concam(camID2)
                    if camera2 == 0:
                        nRet = stream1.contents.stopGrabbing(stream1)  # 停止拉流
                        eventSubscribe = pointer(GENICAM_EventSubscribe())  # 反注册相机连接状态回调
                        eventSubscribeInfo = GENICAM_EventSubscribeInfo()
                        eventSubscribeInfo.pCamera = pointer(camera1)
                        nRet = GENICAM_createEventSubscribe(byref(eventSubscribeInfo), byref(eventSubscribe))
                        nRet = eventSubscribe.contents.unsubscribeConnectArgsEx(eventSubscribe, FuncEx1, cameraInfo1)
                        return
                elif vali.get() == 3:  # 打开三个相机
                    camera2, stream2, FuncEx2, cameraInfo2 = concam(camID2)
                    camera3, stream3, FuncEx3, cameraInfo3 = concam(camID3)
                    if camera2 == 0 or camera3 == 0:
                        nRet = stream1.contents.stopGrabbing(stream1)  # 停止拉流
                        eventSubscribe = pointer(GENICAM_EventSubscribe())  # 反注册相机连接状态回调
                        eventSubscribeInfo = GENICAM_EventSubscribeInfo()
                        eventSubscribeInfo.pCamera = pointer(camera1)
                        nRet = GENICAM_createEventSubscribe(byref(eventSubscribeInfo), byref(eventSubscribe))
                        nRet = eventSubscribe.contents.unsubscribeConnectArgsEx(eventSubscribe, FuncEx1, cameraInfo1)
                    if camera2 == 0:
                        if camera3 != 0:
                            nRet = stream1.contents.stopGrabbing(stream3)  # 停止拉流
                            eventSubscribe = pointer(GENICAM_EventSubscribe())  # 反注册相机连接状态回调
                            eventSubscribeInfo = GENICAM_EventSubscribeInfo()
                            eventSubscribeInfo.pCamera = pointer(camera3)
                            nRet = GENICAM_createEventSubscribe(byref(eventSubscribeInfo), byref(eventSubscribe))
                            nRet = eventSubscribe.contents.unsubscribeConnectArgsEx(eventSubscribe, FuncEx1,cameraInfo3)
                        return
                    elif camera3 == 0:
                        if camera2 != 0:
                            nRet = stream1.contents.stopGrabbing(stream2)  # 停止拉流
                            eventSubscribe = pointer(GENICAM_EventSubscribe())  # 反注册相机连接状态回调
                            eventSubscribeInfo = GENICAM_EventSubscribeInfo()
                            eventSubscribeInfo.pCamera = pointer(camera2)
                            nRet = GENICAM_createEventSubscribe(byref(eventSubscribeInfo), byref(eventSubscribe))
                            nRet = eventSubscribe.contents.unsubscribeConnectArgsEx(eventSubscribe, FuncEx2, cameraInfo2)
                        return
            bt0.config(text=r"关闭相机")
            mousetype = 1
            killcam = 0
            photocam = []
            a = 0
            rd1.config(state='disabled')
            rd2.config(state='disabled')
            rd3.config(state='disabled')
            cob0.config(state='disabled')
            if vali.get() == 1: #打开一个相机
                while True:
                    time.sleep(0.04)
                    if photocam:
                        mousetype = 1.5
                        if photocam[0] == 'all':
                            picnum = 0
                        elif 'c' in photocam[0]:
                            picnum = photocam[0][1]
                        elif 'r' in photocam[0]:
                            picnum = int(photocam[0][1]) + 10
                        elif 'u' == photocam[0]:
                            picnum = 101
                        elif 'v' == photocam[0]:
                            picnum = 102
                        elif 'u1' == photocam[0]:
                            picnum,photocam[0] = 103,'u'
                        elif 'v1' == photocam[0]:
                            picnum,photocam[0] = 104,'v'
                        elif 'white' == photocam[0]:
                            picnum = 105
                        else:
                            picnum = 'photo'
                        msg = sendmsg(clientweival.get(),clientheival.get(),columnval.get(),rowval.get(),picnum,radiusval.get(),restore)
                        if picnum != 'photo' and tcpsult == 0:
                            client_send.sendall(bytes(msg, encoding="utf-8"))
                        a += 1
                        time.sleep(delayval.get()-1)
                    if cammodel.get() == "IDS":
                        if changegainexposure == 1:
                            exp_time = ueye.c_double(float(exposureval.get()))  # 设置曝光时间
                            ueye.is_SetHWGainFactor(hCam1, ueye.IS_INQUIRE_MASTER_GAIN_FACTOR, 300)
                            ueye.is_SetHWGainFactor(hCam1, ueye.IS_SET_MASTER_GAIN_FACTOR, gainval.get())
                            ueye.is_Exposure(hCam1, ueye.IS_EXPOSURE_CMD_SET_EXPOSURE, exp_time, 8)
                            changegainexposure = 0
                        array = ueye.get_data(pcImageMemory1, camwid1,camhei1, nBitsPerPixel1, pitch1, copy=False)
                        cvImage = np.reshape(array, (camhei1.value, camwid1.value, bytes_per_pixel1))
                    else:
                        if changegainexposure == 1:
                            setExposureTime(camera1, exposureval.get() * (1000000 - 16) / 226 + 16)
                            setGain(camera1, (gainval.get()-100) * 31 / 100 +1)
                            changegainexposure = 0
                        frame = pointer(GENICAM_Frame())
                        nRet = stream1.contents.getFrame(stream1, byref(frame), c_uint(1000))
                        if nRet != 0:
                            showtext("获取图像失败！超时：[1000]毫秒", 'red')
                            stream1.contents.release(stream1)  # 释放相关资源
                            break
                        nRet = frame.contents.valid(frame)
                        if nRet != 0:
                            showtext("框架无效!", 'red')
                            frame.contents.release(frame)  # 释放驱动图像缓存资源
                            stream1.contents.release(stream1)  # 释放相关资源
                            break
                        imageParamsdataSize = frame.contents.getImageSize(frame)
                        imageParamsheight = frame.contents.getImageHeight(frame)
                        imageParamswidth = frame.contents.getImageWidth(frame)
                        imageBuff = frame.contents.getImage(frame)  # 将裸数据图像拷出
                        userBuff = c_buffer(b'\0', imageParamsdataSize)
                        memmove(userBuff, c_char_p(imageBuff), imageParamsdataSize)
                        frame.contents.release(frame)  # 释放驱动图像缓存
                        grayByteArray = bytearray(userBuff)
                        cvImage = np.array(grayByteArray).reshape(imageParamsheight, imageParamswidth)
                    frames = cv2.resize(cvImage, (camw1,camh1))
                    if a == 0:
                        tkImage1 = ImageTk.PhotoImage(PIL.Image.fromarray(frames))
                        canvas.create_image(0,0,anchor='nw',image=tkImage1)
                    else:
                        tkImage1.paste(PIL.Image.fromarray(frames))
                    if len(markpoint) > 0:
                        for i in markpoint:
                            canvas.create_text(i[1],i[2],text=str(i[0]),fill='yellow')
                            canvas.create_rectangle(i[1]-10,i[2]-10,i[1]+10,i[2]+10,outline='red')
                    win.update()
                    win.after(1)
                    if a != 0:
                        if photocam[0] == 'photo':
                            saveimg(cvImage,paths+r'\%s'%(projectname))+r'\%s.png'%(photocam[0])
                        elif photocam[0] == 'm':
                            saveimg(cvImage,mkdir(paths+r'\%s\mark1'%(projectname))+r'\Mark.png')
                        else:
                            saveimg(cvImage,mkdir(paths+r'\%s\%s_%s\%scam1'%(projectname,pcIP,port,restore))+ r'\%s.png'%(photocam[0]))
                        del photocam[0]
                        if len(photocam) == 0:
                            mousetype = 1
                            msg = sendmsg(clientweival.get(),clientheival.get(),columnval.get(),rowval.get(),-1,radiusval.get(),restore)
                            if picnum != 'photo' and tcpsult == 0: #黑屏
                                client_send.sendall(bytes(msg, encoding="utf-8"))
                                client_send.close()
                    if photocam:
                        time.sleep(1)
                    else:
                        a = 0
                    if killcam == 27: # 如果要关闭相机，请按ESC
                        break
                if cammodel.get() == "IDS":
                    ueye.is_FreeImageMem(hCam1, pcImageMemory1, MemID1)
                    ueye.is_ExitCamera(hCam1)
                else:
                    nRet = stream1.contents.stopGrabbing(stream1)  # 停止拉流
                    if nRet != 0:
                        showtext("停止拉流失败!", 'red')
                        stream1.contents.release(stream1)  # 释放相关资源
                        return
                    eventSubscribe = pointer(GENICAM_EventSubscribe())  # 反注册相机连接状态回调
                    eventSubscribeInfo = GENICAM_EventSubscribeInfo()
                    eventSubscribeInfo.pCamera = pointer(camera1)
                    nRet = GENICAM_createEventSubscribe(byref(eventSubscribeInfo), byref(eventSubscribe))
                    if nRet != 0:
                        showtext("创建事件订阅失败!", 'red')
                        return
                    nRet = eventSubscribe.contents.unsubscribeConnectArgsEx(eventSubscribe,FuncEx1,cameraInfo1)
                    if nRet != 0:
                        showtext("取消订阅失败!", 'red')
                        eventSubscribe.contents.release(eventSubscribe)  # 不再使用时，需释放相关资源
                        return
                    eventSubscribe.contents.release(eventSubscribe)  # 不再使用时，需释放相关资源
                    nRet = camera1.disConnect(byref(camera1))
                    if nRet != 0:
                        showtext("关闭相机失败!", 'red')
                        stream1.contents.release(stream1)  # 释放相关资源
                        return
                    stream1.contents.release(stream1)  # 释放相关资源
                canvas.delete('all')
            elif vali.get() == 2: #打开两个相机
                while True:
                    time.sleep(0.04)
                    if photocam:
                        mousetype = 1.5
                        if photocam[0] == 'all':
                            picnum = 0
                        elif 'c' in photocam[0]:
                            picnum = photocam[0][1]
                        elif 'r' in photocam[0]:
                            picnum = int(photocam[0][1]) + 10
                        elif 'u' == photocam[0]:
                            picnum = 101
                        elif 'v' == photocam[0]:
                            picnum = 102
                        elif 'u1' == photocam[0]:
                            picnum,photocam[0] = 103,'u'
                        elif 'v1' == photocam[0]:
                            picnum,photocam[0] = 104,'v'
                        elif 'white' == photocam[0]:
                            picnum = 105
                        else:
                            picnum = 'photo'
                        msg = sendmsg(clientweival.get(),clientheival.get(),columnval.get(),rowval.get(),picnum,radiusval.get(),restore)
                        if picnum != 'photo' and tcpsult == 0:
                            client_send.sendall(bytes(msg, encoding="utf-8"))
                        a += 1
                        time.sleep(delayval.get() - 1)
                    if cammodel.get() == "IDS":
                        if changegainexposure == 1:
                            exp_time = ueye.c_double(float(exposureval.get()))  # 设置曝光时间
                            ueye.is_SetHWGainFactor(hCam1, ueye.IS_INQUIRE_MASTER_GAIN_FACTOR, 300)
                            ueye.is_SetHWGainFactor(hCam2, ueye.IS_INQUIRE_MASTER_GAIN_FACTOR, 300)
                            ueye.is_SetHWGainFactor(hCam1, ueye.IS_SET_MASTER_GAIN_FACTOR, gainval.get())
                            ueye.is_SetHWGainFactor(hCam2, ueye.IS_SET_MASTER_GAIN_FACTOR, gainval.get())
                            ueye.is_Exposure(hCam1, ueye.IS_EXPOSURE_CMD_SET_EXPOSURE, exp_time, 8)
                            ueye.is_Exposure(hCam2, ueye.IS_EXPOSURE_CMD_SET_EXPOSURE, exp_time, 8)
                            changegainexposure = 0
                        array1 = ueye.get_data(pcImageMemory1, camwid1, camhei1, nBitsPerPixel1, pitch1, copy=False)
                        array2 = ueye.get_data(pcImageMemory2, camwid2, camhei2, nBitsPerPixel2, pitch2, copy=False)
                        cvImage1 = np.reshape(array1, (camhei1.value, camwid1.value, bytes_per_pixel1))
                        cvImage2 = np.reshape(array2, (camhei2.value, camwid2.value, bytes_per_pixel2))
                    else:
                        if changegainexposure == 1:
                            setExposureTime(camera1, exposureval.get() * (1000000 - 16) / 226 + 16)
                            setExposureTime(camera2, exposureval.get() * (1000000 - 16) / 226 + 16)
                            setGain(camera1, (gainval.get() - 100) * 31 / 100 + 1)
                            setGain(camera2, (gainval.get() - 100) * 31 / 100 + 1)
                            changegainexposure = 0
                        frame1 = pointer(GENICAM_Frame())
                        frame2 = pointer(GENICAM_Frame())
                        nRet = stream1.contents.getFrame(stream1, byref(frame1), c_uint(1000))
                        nRet = stream2.contents.getFrame(stream2, byref(frame2), c_uint(1000))
                        if nRet != 0:
                            showtext("获取图像失败！超时：[1000]毫秒", 'red')
                            stream1.contents.release(stream1)  # 释放相关资源
                            stream2.contents.release(stream2)  # 释放相关资源
                            break
                        nRet = frame1.contents.valid(frame1)
                        nRet = frame2.contents.valid(frame2)
                        if nRet != 0:
                            showtext("框架无效!", 'red')
                            frame1.contents.release(frame1)  # 释放驱动图像缓存资源
                            frame2.contents.release(frame2)  # 释放驱动图像缓存资源
                            stream1.contents.release(stream1)  # 释放相关资源
                            stream2.contents.release(stream2)  # 释放相关资源
                            break
                        # imageParams1 = IMGCNV_SOpenParam()  # 给转码所需的参数赋值
                        # imageParams2 = IMGCNV_SOpenParam()  # 给转码所需的参数赋值
                        imageParams1dataSize = frame1.contents.getImageSize(frame1)
                        imageParams2dataSize = frame2.contents.getImageSize(frame2)
                        imageParams1height = frame1.contents.getImageHeight(frame1)
                        imageParams2height = frame2.contents.getImageHeight(frame2)
                        imageParams1width = frame1.contents.getImageWidth(frame1)
                        imageParams2width = frame2.contents.getImageWidth(frame2)
                        # imageParams1.paddingX = frame1.contents.getImagePaddingX(frame1)
                        # imageParams2.paddingX = frame2.contents.getImagePaddingX(frame2)
                        # imageParams1.paddingY = frame1.contents.getImagePaddingY(frame1)
                        # imageParams2.paddingY = frame2.contents.getImagePaddingY(frame2)
                        # imageParams1.pixelForamt = frame1.contents.getImagePixelFormat(frame1)
                        # imageParams2.pixelForamt = frame2.contents.getImagePixelFormat(frame2)
                        imageBuff1 = frame1.contents.getImage(frame1)  # 将裸数据图像拷出
                        imageBuff2 = frame2.contents.getImage(frame2)  # 将裸数据图像拷出
                        userBuff1 = c_buffer(b'\0', imageParams1dataSize)
                        userBuff2 = c_buffer(b'\0', imageParams2dataSize)
                        memmove(userBuff1, c_char_p(imageBuff1), imageParams1dataSize)
                        memmove(userBuff2, c_char_p(imageBuff2), imageParams2dataSize)
                        frame1.contents.release(frame1)  # 释放驱动图像缓存
                        frame2.contents.release(frame2)  # 释放驱动图像缓存
                        grayByteArray1 = bytearray(userBuff1)
                        grayByteArray2 = bytearray(userBuff2)
                        cvImage1 = np.array(grayByteArray1).reshape(imageParams1height, imageParams1width)
                        cvImage2 = np.array(grayByteArray2).reshape(imageParams2height, imageParams2width)
                    frames1 = cv2.resize(cvImage1, (camw1,camh1))
                    frames2 = cv2.resize(cvImage2, (camw2,camh2))
                    if a == 0:
                        tkImage1 = ImageTk.PhotoImage(PIL.Image.fromarray(frames1))
                        tkImage2 = ImageTk.PhotoImage(PIL.Image.fromarray(frames2))
                        canvas.create_image(0, 0, anchor='nw', image=tkImage1)
                        canvas2.create_image(0, 0, anchor='nw', image=tkImage2)
                    else:
                        tkImage1.paste(PIL.Image.fromarray(frames1))
                        tkImage2.paste(PIL.Image.fromarray(frames2))
                    win.update()
                    win.after(1)
                    if a != 0:
                        if photocam[0] == 'photo':
                            saveimg(cvImage1,mkdir(paths+r'\%s'%(projectname))+r'\%s1.png'%(photocam[0]))
                            saveimg(cvImage2,mkdir(paths+r'\%s'%(projectname))+r'\%s2.png'%(photocam[0]))
                        elif photocam[0] == 'm':
                            saveimg(cvImage1,mkdir(paths+r'\%s\mark1'%(projectname))+r'\Mark.png')
                            saveimg(cvImage2,mkdir(paths+r'\%s\mark2'%(projectname))+r'\Mark.png')
                        else:
                            saveimg(cvImage1,mkdir(paths+r'\%s\%s_%s\%scam1'%(projectname,pcIP,port,restore))+ r'\%s.png'%(photocam[0]))
                            saveimg(cvImage2,mkdir(paths+r'\%s\%s_%s\%scam2'%(projectname,pcIP,port,restore))+ r'\%s.png'%(photocam[0]))
                        del photocam[0]
                        if len(photocam) == 0:
                            mousetype = 1
                            msg = sendmsg(clientweival.get(),clientheival.get(),columnval.get(),rowval.get(),-1,radiusval.get(),restore)
                            if picnum != 'photo' and tcpsult ==0: #黑屏
                                client_send.sendall(bytes(msg, encoding="utf-8"))
                                client_send.close()
                    if photocam:
                        time.sleep(1)
                    else:
                        a = 0
                    if killcam == 27: # 如果要关闭相机，请按ESC
                        break
                canvas.delete('all')
                canvas2.delete('all')
                if cammodel.get() == "IDS":
                    ueye.is_FreeImageMem(hCam1, pcImageMemory1, MemID1)
                    ueye.is_FreeImageMem(hCam2, pcImageMemory2, MemID2)
                    ueye.is_ExitCamera(hCam1)
                    ueye.is_ExitCamera(hCam2)
                else:
                    nRet = stream1.contents.stopGrabbing(stream1)  # 停止拉流
                    nRet = stream2.contents.stopGrabbing(stream2)  # 停止拉流
                    if nRet != 0:
                        showtext("停止拉流失败!", 'red')
                        stream1.contents.release(stream1)  # 释放相关资源
                        stream2.contents.release(stream2)  # 释放相关资源
                        return
                    eventSubscribe1 = pointer(GENICAM_EventSubscribe())  # 反注册相机连接状态回调
                    eventSubscribe2 = pointer(GENICAM_EventSubscribe())  # 反注册相机连接状态回调
                    eventSubscribeInfo1 = GENICAM_EventSubscribeInfo()
                    eventSubscribeInfo2 = GENICAM_EventSubscribeInfo()
                    eventSubscribeInfo1.pCamera = pointer(camera1)
                    eventSubscribeInfo2.pCamera = pointer(camera2)
                    nRet = GENICAM_createEventSubscribe(byref(eventSubscribeInfo1), byref(eventSubscribe1))
                    nRet = GENICAM_createEventSubscribe(byref(eventSubscribeInfo2), byref(eventSubscribe2))
                    if nRet != 0:
                        showtext("创建事件订阅失败!", 'red')
                        return
                    nRet = eventSubscribe1.contents.unsubscribeConnectArgsEx(eventSubscribe1, FuncEx1, cameraInfo1)
                    nRet = eventSubscribe2.contents.unsubscribeConnectArgsEx(eventSubscribe2, FuncEx2, cameraInfo2)
                    if nRet != 0:
                        showtext("取消订阅失败!", 'red')
                        eventSubscribe1.contents.release(eventSubscribe1)  # 不再使用时，需释放相关资源
                        eventSubscribe2.contents.release(eventSubscribe2)  # 不再使用时，需释放相关资源
                        return
                    eventSubscribe1.contents.release(eventSubscribe1)  # 不再使用时，需释放相关资源
                    eventSubscribe2.contents.release(eventSubscribe2)  # 不再使用时，需释放相关资源
                    nRet = camera1.disConnect(byref(camera1))
                    nRet = camera2.disConnect(byref(camera2))
                    if nRet != 0:
                        showtext("关闭相机失败!", 'red')
                        stream1.contents.release(stream1)  # 释放相关资源
                        stream2.contents.release(stream2)  # 释放相关资源
                        return
                    stream1.contents.release(stream1)  # 释放相关资源
                    stream2.contents.release(stream2)  # 释放相关资源
            elif vali.get() == 3: #打开三个相机
                while True:
                    time.sleep(0.04)
                    if photocam:
                        mousetype = 1.5
                        if photocam[0] == 'all':
                            picnum = 0
                        elif 'c' in photocam[0]:
                            picnum = photocam[0][1]
                        elif 'r' in photocam[0]:
                            picnum = int(photocam[0][1]) + 10
                        elif 'u' == photocam[0]:
                            picnum = 101
                        elif 'v' == photocam[0]:
                            picnum = 102
                        elif 'u1' == photocam[0]:
                            picnum,photocam[0] = 103,'u'
                        elif 'v1' == photocam[0]:
                            picnum,photocam[0] = 104,'v'
                        elif 'white' == photocam[0]:
                            picnum = 105
                        else:
                            picnum = 'photo'
                        msg = sendmsg(clientweival.get(),clientheival.get(),columnval.get(),rowval.get(),picnum,radiusval.get(),restore)
                        if picnum != 'photo' and tcpsult == 0:
                            client_send.sendall(bytes(msg, encoding="utf-8"))
                        a += 1
                        time.sleep(delayval.get() - 1)
                    if cammodel.get() == "IDS":
                        if changegainexposure == 1:
                            exp_time = ueye.c_double(float(exposureval.get()))  # 设置曝光时间
                            ueye.is_SetHWGainFactor(hCam1, ueye.IS_INQUIRE_MASTER_GAIN_FACTOR, 300)
                            ueye.is_SetHWGainFactor(hCam2, ueye.IS_INQUIRE_MASTER_GAIN_FACTOR, 300)
                            ueye.is_SetHWGainFactor(hCam3, ueye.IS_INQUIRE_MASTER_GAIN_FACTOR, 300)
                            ueye.is_SetHWGainFactor(hCam1, ueye.IS_SET_MASTER_GAIN_FACTOR, gainval.get())
                            ueye.is_SetHWGainFactor(hCam2, ueye.IS_SET_MASTER_GAIN_FACTOR, gainval.get())
                            ueye.is_SetHWGainFactor(hCam3, ueye.IS_SET_MASTER_GAIN_FACTOR, gainval.get())
                            ueye.is_Exposure(hCam1, ueye.IS_EXPOSURE_CMD_SET_EXPOSURE, exp_time, 8)
                            ueye.is_Exposure(hCam2, ueye.IS_EXPOSURE_CMD_SET_EXPOSURE, exp_time, 8)
                            ueye.is_Exposure(hCam3, ueye.IS_EXPOSURE_CMD_SET_EXPOSURE, exp_time, 8)
                            changegainexposure = 0
                        array1 = ueye.get_data(pcImageMemory1, camwid1, camhei1, nBitsPerPixel1, pitch1, copy=False)
                        array2 = ueye.get_data(pcImageMemory2, camwid2, camhei2, nBitsPerPixel2, pitch2, copy=False)
                        array3 = ueye.get_data(pcImageMemory3, camwid3, camhei3, nBitsPerPixel3, pitch3, copy=False)
                        cvImage1 = np.reshape(array1, (camhei1.value, camwid1.value, bytes_per_pixel1))
                        cvImage2 = np.reshape(array2, (camhei2.value, camwid2.value, bytes_per_pixel2))
                        cvImage3 = np.reshape(array3, (camhei3.value, camwid3.value, bytes_per_pixel3))
                    else:
                        if changegainexposure == 1:
                            setExposureTime(camera1, exposureval.get() * (1000000 - 16) / 226 + 16)
                            setExposureTime(camera2, exposureval.get() * (1000000 - 16) / 226 + 16)
                            setExposureTime(camera3, exposureval.get() * (1000000 - 16) / 226 + 16)
                            setGain(camera1, (gainval.get() - 100) * 31 / 100 + 1)
                            setGain(camera2, (gainval.get() - 100) * 31 / 100 + 1)
                            setGain(camera3, (gainval.get() - 100) * 31 / 100 + 1)
                            changegainexposure = 0
                        frame1 = pointer(GENICAM_Frame())
                        frame2 = pointer(GENICAM_Frame())
                        frame3 = pointer(GENICAM_Frame())
                        nRet = stream1.contents.getFrame(stream1, byref(frame1), c_uint(1000))
                        nRet = stream2.contents.getFrame(stream2, byref(frame2), c_uint(1000))
                        nRet = stream3.contents.getFrame(stream3, byref(frame3), c_uint(1000))
                        if nRet != 0:
                            showtext("获取图像失败！超时：[1000]毫秒", 'red')
                            stream1.contents.release(stream1)  # 释放相关资源
                            stream2.contents.release(stream2)  # 释放相关资源
                            stream3.contents.release(stream3)  # 释放相关资源
                            break
                        nRet = frame1.contents.valid(frame1)
                        nRet = frame2.contents.valid(frame2)
                        nRet = frame3.contents.valid(frame3)
                        if nRet != 0:
                            showtext("框架无效!", 'red')
                            frame1.contents.release(frame1)  # 释放驱动图像缓存资源
                            frame2.contents.release(frame2)  # 释放驱动图像缓存资源
                            frame3.contents.release(frame3)  # 释放驱动图像缓存资源
                            stream1.contents.release(stream1)  # 释放相关资源
                            stream2.contents.release(stream2)  # 释放相关资源
                            stream3.contents.release(stream3)  # 释放相关资源
                            break
                        # imageParams1 = IMGCNV_SOpenParam()  # 给转码所需的参数赋值
                        # imageParams2 = IMGCNV_SOpenParam()  # 给转码所需的参数赋值
                        # imageParams3 = IMGCNV_SOpenParam()  # 给转码所需的参数赋值
                        imageParams1dataSize = frame1.contents.getImageSize(frame1)
                        imageParams2dataSize = frame2.contents.getImageSize(frame2)
                        imageParams3dataSize = frame3.contents.getImageSize(frame3)
                        imageParams1height = frame1.contents.getImageHeight(frame1)
                        imageParams2height = frame2.contents.getImageHeight(frame2)
                        imageParams3height = frame3.contents.getImageHeight(frame3)
                        imageParams1width = frame1.contents.getImageWidth(frame1)
                        imageParams2width = frame2.contents.getImageWidth(frame2)
                        imageParams3width = frame3.contents.getImageWidth(frame3)
                        # imageParams1.paddingX = frame1.contents.getImagePaddingX(frame1)
                        # imageParams2.paddingX = frame2.contents.getImagePaddingX(frame2)
                        # imageParams3.paddingX = frame3.contents.getImagePaddingX(frame3)
                        # imageParams1.paddingY = frame1.contents.getImagePaddingY(frame1)
                        # imageParams2.paddingY = frame2.contents.getImagePaddingY(frame2)
                        # imageParams3.paddingY = frame3.contents.getImagePaddingY(frame3)
                        # imageParams1.pixelForamt = frame1.contents.getImagePixelFormat(frame1)
                        # imageParams2.pixelForamt = frame2.contents.getImagePixelFormat(frame2)
                        # imageParams3.pixelForamt = frame3.contents.getImagePixelFormat(frame3)
                        imageBuff1 = frame1.contents.getImage(frame1)  # 将裸数据图像拷出
                        imageBuff2 = frame2.contents.getImage(frame2)  # 将裸数据图像拷出
                        imageBuff3 = frame3.contents.getImage(frame3)  # 将裸数据图像拷出
                        userBuff1 = c_buffer(b'\0', imageParams1dataSize)
                        userBuff2 = c_buffer(b'\0', imageParams2dataSize)
                        userBuff3 = c_buffer(b'\0', imageParams3dataSize)
                        memmove(userBuff1, c_char_p(imageBuff1), imageParams1dataSize)
                        memmove(userBuff2, c_char_p(imageBuff2), imageParams2dataSize)
                        memmove(userBuff3, c_char_p(imageBuff3), imageParams3dataSize)
                        frame1.contents.release(frame1)  # 释放驱动图像缓存
                        frame2.contents.release(frame2)  # 释放驱动图像缓存
                        frame3.contents.release(frame3)  # 释放驱动图像缓存
                        grayByteArray1 = bytearray(userBuff1)
                        grayByteArray2 = bytearray(userBuff2)
                        grayByteArray3 = bytearray(userBuff3)
                        cvImage1 = np.array(grayByteArray1).reshape(imageParams1height, imageParams1width)
                        cvImage2 = np.array(grayByteArray2).reshape(imageParams2height, imageParams2width)
                        cvImage3 = np.array(grayByteArray3).reshape(imageParams3height, imageParams3width)
                    frames1 = cv2.resize(cvImage1, (camw1,camh1))
                    frames2 = cv2.resize(cvImage2, (camw2,camh2))
                    frames3 = cv2.resize(cvImage3, (camw3,camh3))
                    if a == 0:
                        tkImage1 = ImageTk.PhotoImage(PIL.Image.fromarray(frames1))
                        tkImage2 = ImageTk.PhotoImage(PIL.Image.fromarray(frames2))
                        tkImage3 = ImageTk.PhotoImage(PIL.Image.fromarray(frames3))
                        canvas.create_image(0, 0, anchor='nw', image=tkImage1)
                        canvas2.create_image(0, 0, anchor='nw', image=tkImage2)
                        canvas3.create_image(0, 0, anchor='nw', image=tkImage3)
                    else:
                        tkImage1.paste(PIL.Image.fromarray(frames1))
                        tkImage2.paste(PIL.Image.fromarray(frames2))
                        tkImage3.paste(PIL.Image.fromarray(frames3))
                    win.update()
                    win.after(1)
                    if a != 0:
                        if photocam[0] == 'photo':
                            saveimg(cvImage1,mkdir(paths+r'\%s'%(projectname))+r'\%s1.png'%(photocam[0]))
                            saveimg(cvImage2,mkdir(paths+r'\%s'%(projectname))+r'\%s2.png'%(photocam[0]))
                            saveimg(cvImage3,mkdir(paths+r'\%s'%(projectname))+r'\%s3.png'%(photocam[0]))
                        elif photocam[0] == 'm':
                            saveimg(cvImage1,mkdir(paths+r'\%s\mark1'%(projectname))+r'\Mark.png')
                            saveimg(cvImage2,mkdir(paths+r'\%s\mark2'%(projectname))+r'\Mark.png')
                            saveimg(cvImage3,mkdir(paths+r'\%s\mark3'%(projectname))+r'\Mark.png')
                        else:
                            saveimg(cvImage1,mkdir(paths+r'\%s\%s_%s\%scam1'%(projectname,pcIP,port,restore))+ r'\%s.png'%(photocam[0]))
                            saveimg(cvImage2,mkdir(paths+r'\%s\%s_%s\%scam2'%(projectname,pcIP,port,restore))+ r'\%s.png'%(photocam[0]))
                            saveimg(cvImage3,mkdir(paths+r'\%s\%s_%s\%scam3'%(projectname,pcIP,port,restore))+ r'\%s.png'%(photocam[0]))
                        del photocam[0]
                        if len(photocam) == 0:
                            mousetype = 1
                            msg = sendmsg(clientweival.get(),clientheival.get(),columnval.get(),rowval.get(),-1,radiusval.get(),restore)
                            if picnum != 'photo' and tcpsult == 0: #黑屏
                                client_send.sendall(bytes(msg, encoding="utf-8"))
                                client_send.close()
                    if photocam:
                        time.sleep(1)
                    else:
                        a = 0
                    if killcam == 27: # 如果要关闭相机，请按ESC
                        break
                canvas.delete('all')
                canvas2.delete('all')
                canvas3.delete('all')
                if cammodel.get() == "IDS":
                    ueye.is_FreeImageMem(hCam1, pcImageMemory1, MemID1)
                    ueye.is_FreeImageMem(hCam2, pcImageMemory2, MemID2)
                    ueye.is_FreeImageMem(hCam3, pcImageMemory3, MemID3)
                    ueye.is_ExitCamera(hCam1)
                    ueye.is_ExitCamera(hCam2)
                    ueye.is_ExitCamera(hCam3)
                else:
                    nRet = stream1.contents.stopGrabbing(stream1)  # 停止拉流
                    nRet = stream2.contents.stopGrabbing(stream2)  # 停止拉流
                    nRet = stream3.contents.stopGrabbing(stream3)  # 停止拉流
                    if nRet != 0:
                        showtext("停止拉流失败!", 'red')
                        stream1.contents.release(stream1)  # 释放相关资源
                        stream2.contents.release(stream2)  # 释放相关资源
                        stream3.contents.release(stream3)  # 释放相关资源
                        return
                    eventSubscribe1 = pointer(GENICAM_EventSubscribe())  # 反注册相机连接状态回调
                    eventSubscribe2 = pointer(GENICAM_EventSubscribe())  # 反注册相机连接状态回调
                    eventSubscribe3 = pointer(GENICAM_EventSubscribe())  # 反注册相机连接状态回调
                    eventSubscribeInfo1 = GENICAM_EventSubscribeInfo()
                    eventSubscribeInfo2 = GENICAM_EventSubscribeInfo()
                    eventSubscribeInfo3 = GENICAM_EventSubscribeInfo()
                    eventSubscribeInfo1.pCamera = pointer(camera1)
                    eventSubscribeInfo2.pCamera = pointer(camera2)
                    eventSubscribeInfo3.pCamera = pointer(camera3)
                    nRet = GENICAM_createEventSubscribe(byref(eventSubscribeInfo1), byref(eventSubscribe1))
                    nRet = GENICAM_createEventSubscribe(byref(eventSubscribeInfo2), byref(eventSubscribe2))
                    nRet = GENICAM_createEventSubscribe(byref(eventSubscribeInfo3), byref(eventSubscribe3))
                    if nRet != 0:
                        showtext("创建事件订阅失败!", 'red')
                        return
                    nRet = eventSubscribe1.contents.unsubscribeConnectArgsEx(eventSubscribe1, FuncEx1, cameraInfo1)
                    nRet = eventSubscribe2.contents.unsubscribeConnectArgsEx(eventSubscribe2, FuncEx2, cameraInfo2)
                    nRet = eventSubscribe3.contents.unsubscribeConnectArgsEx(eventSubscribe3, FuncEx3, cameraInfo3)
                    if nRet != 0:
                        showtext("取消订阅失败!", 'red')
                        eventSubscribe1.contents.release(eventSubscribe1)  # 不再使用时，需释放相关资源
                        eventSubscribe2.contents.release(eventSubscribe2)  # 不再使用时，需释放相关资源
                        eventSubscribe3.contents.release(eventSubscribe3)  # 不再使用时，需释放相关资源
                        return
                    eventSubscribe1.contents.release(eventSubscribe1)  # 不再使用时，需释放相关资源
                    eventSubscribe2.contents.release(eventSubscribe2)  # 不再使用时，需释放相关资源
                    eventSubscribe3.contents.release(eventSubscribe3)  # 不再使用时，需释放相关资源
                    nRet = camera1.disConnect(byref(camera1))
                    nRet = camera2.disConnect(byref(camera2))
                    nRet = camera3.disConnect(byref(camera3))
                    if nRet != 0:
                        showtext("关闭相机失败!", 'red')
                        stream1.contents.release(stream1)  # 释放相关资源
                        stream2.contents.release(stream2)  # 释放相关资源
                        stream3.contents.release(stream3)  # 释放相关资源
                        return
                    stream1.contents.release(stream1)  # 释放相关资源
                    stream2.contents.release(stream2)  # 释放相关资源
                    stream3.contents.release(stream3)  # 释放相关资源
            mousetype = 0
            rd1.config(state='normal')
            rd2.config(state='normal')
            rd3.config(state='normal')
            cob0.config(state='normal')
            bt0.config(text=r"打开相机")
        elif mousetype == 1:
            killcam = 27
    except Exception as e:
        showtext(str(e.__traceback__.tb_lineno)+str(e), 'red')
# 银幕/mark点拍照
def photo(mark):
    global photocam, pcIP, port
    if len(tree.focus()) != 0:
        if mark == 1: #mark点拍照
            photocam = ['m']
        else:
            if 'sceen' not in tree.focus():
                pcIP = tree.item(tree.focus(), 'values')[0][:-6]
                port = tree.item(tree.focus(), 'values')[0][-5:]
                photocam = ['0']
            else:
                photocam = ['photo']
# 普通客户端发送
def sendmsg(wei,hei,colum,row,num,radius,res):
    if res == 'Grid':
        grid = 1
    else:
        grid = 0
    if num == 105:
        msg = '{"Resoulution":"","Type":"CAMERA","P_number":0,"IP_PORT":"","Markpoints":[],"Commen_Sender":"%s,%s,%s,%s,%s,13,1,0,2",' \
              '"Fix_List":[],"Other_Sender":"","Sj_basic":[],"Sj_detil1":[],"Sj_detil2":[],"Sj_point_basic":[]}' % (wei, hei, colum, row, num)
    else:
        msg = '{"Resoulution":"","Type":"CAMERA","P_number":0,"IP_PORT":"","Markpoints":[],"Commen_Sender":"%s,%s,%s,%s,%s,%s,%s,0,0",' \
              '"Fix_List":[],"Other_Sender":"","Sj_basic":[],"Sj_detil1":[],"Sj_detil2":[],"Sj_point_basic":[]}' % (wei,hei,colum,row,num,radius,grid)
    return msg
# 发送客户端返回行列数
def sendmsg2(wei,hei,colum,row,num,radius,res):
    if res == 'Grid':
        grid = 1
    else:
        grid = 0
    msg = '{"Resoulution":"","Type":"CAMERAVAR","P_number":0,"IP_PORT":"","Markpoints":[],"Commen_Sender":"%s,%s,%s,%s,%s,%s,%s,0,0",' \
          '"Fix_List":[],"Other_Sender":"","Sj_basic":[],"Sj_detil1":[],"Sj_detil2":[],"Sj_point_basic":[]}' % (wei,hei,colum,row,num,radius,grid)
    return msg
# 显示圆圈/黑屏
def showcircle(gi):
    global pcIP, port, client_send
    try:
        if len(tree.focus()) != 0:
            if 'sceen' in tree.focus():
                for i in tree.get_children(tree.focus()):
                    pcIP = tree.item(i,'values')[0][:-6]
                    port = tree.item(i,'values')[0][-5:]
                    if gi == 0: #标准点
                        msg = sendmsg(clientweival.get(),clientheival.get(),columnval.get(),rowval.get(),0,radiusval.get(),'')
                    elif gi == 1: #网格节点
                        msg = sendmsg2(clientweival.get(),clientheival.get(),0,0,0,radiusval.get(),'Grid')
                    elif gi == 2:  # 黑屏
                        msg = sendmsg(clientweival.get(),clientheival.get(),columnval.get(),rowval.get(),-1,radiusval.get(),'')
                    else: #白屏
                        msg = sendmsg(clientweival.get(),clientheival.get(),columnval.get(),rowval.get(),105,radiusval.get(),'')
                    client_send = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 创建socket对象
                    tcpsult = client_send.connect_ex((pcIP, int(port)))  # 建立客户端链接
                    if tcpsult != 0:
                        return
                    client_send.sendall(bytes(msg, encoding="utf-8"))
                    if gi == 1: #网格节点
                        data = client_send.recv(1024).decode()
                        data = data.split('"')[3].split(',')
                        if data != '无数据':
                            gridrow.set(int(data[0]))
                            gridcolumn.set(int(data[1]))
                        else:
                            showtext('未打开三角剖分！', 'red')
                    client_send.close()  # 断开TCP链接
            else:
                pcIP = tree.item(tree.focus(), 'values')[0][:-6]
                port = tree.item(tree.focus(), 'values')[0][-5:]
                if gi == 0: #标准点
                    msg = sendmsg(clientweival.get(),clientheival.get(),columnval.get(),rowval.get(),0,radiusval.get(),'')
                elif gi == 1:  # 网格节点
                    msg = sendmsg2(clientweival.get(),clientheival.get(),0,0,0,radiusval.get(),'Grid')
                elif gi == 2: #黑屏
                    msg = sendmsg(clientweival.get(),clientheival.get(),columnval.get(),rowval.get(),-1,radiusval.get(),'')
                else: #白屏
                    msg = sendmsg(clientweival.get(),clientheival.get(),columnval.get(),rowval.get(),105,radiusval.get(),'')
                client_send = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 创建socket对象
                tcpsult = client_send.connect_ex((pcIP, int(port)))  # 建立客户端链接
                if tcpsult != 0:
                    return
                client_send.sendall(bytes(msg, encoding="utf-8"))
                if gi == 1: # 网格节点
                    data = client_send.recv(1024).decode()
                    data = data.split('"')[3].split(',')
                    if data != '无数据':
                        gridrow.set(int(data[0]))
                        gridcolumn.set(int(data[1]))
                    else:
                        showtext('未打开三角剖分！', 'red')
                client_send.close()  # 断开TCP链接
    except Exception as e:
        showtext(str(e.__traceback__.tb_lineno)+str(e), 'red')
# 线程显示圆圈/黑屏
def run_showcircle(gi):
    try:
        t = threading.Thread(target=showcircle, args=(gi,))
        t.setDaemon(True)
        t.start()
    except Exception as e:
        showtext(str(e.__traceback__.tb_lineno)+str(e), 'red')
# 连续拍照
def seriesphoto(distype):
    global photocam, pcIP, port, client_send,tcpsult,restore
    try:
        if len(tree.focus()) != 0 and 'sceen' not in tree.focus() and mousetype == 1:
            if distype == 0:
                for i in range(1,4):
                    if os.path.exists(pic_path+r'\cam%s\all.png'%(i)):
                        boo = askyesno("提醒", "已存在标准点照片，确定重新拍照？")
                        if boo != True:
                            return
                        else:
                            break
                restore = ''
            elif distype == 1:
                for i in range(1,4):
                    if os.path.exists(pic_path+r'\Gridcam%s\all.png'%(i)):
                        boo = askyesno("提醒", "已存在网格点照片，确定重新拍照？")
                        if boo != True:
                            return
                        else:
                            break
                restore = 'Grid'
            else:
                for i in range(1,4):
                    if os.path.exists(pic_path+r'\Newcam%s\all.png'%(i)):
                        boo = askyesno("提醒", "已存在还原点照片，确定重新拍照？")
                        if boo != True:
                            return
                        else:
                            break
                restore = 'New'
            pcIP = tree.item(tree.focus(), 'values')[0][:-6]
            port = tree.item(tree.focus(), 'values')[0][-5:]
            client_send = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 创建socket对象
            tcpsult = client_send.connect_ex((pcIP, int(port)))  # 建立客户端链接
            if tcpsult !=0:
                showtext('客户端%s：%s连接失败！'%(pcIP,port),'red')
                return
            if distype == 1:  # 网格节点
                msg = sendmsg2(clientweival.get(),clientheival.get(),0,0,0,radiusval.get(),restore)
                client_send.sendall(bytes(msg, encoding="utf-8"))
                data = client_send.recv(1024).decode()
                data = data.split('"')[3].split(',')
                if data != '无数据':
                    column, row = int(data[1]), int(data[0])
                else:
                    showtext('未打开三角剖分！', 'red')
                    return
            else:
                column, row = columnval.get(), rowval.get()
            Pc, Pr = [], []
            for i in range(10):
                if 2 ** i >= column:
                    break
                Pc.append('c%s' % (i + 1))  # 按列拍照名字(不包括总的)
            for i in range(10):
                if 2 ** i >= row:
                    break
                Pr.append('r%s' % (i + 1))  # 按行拍照名字(不包括总的)
            photocam = ['all'] + Pc + Pr
            if distype != 1: #标准点，还原点
                photocam.extend(['u','v','white'])
            else: #网格点
                photocam.extend(['u', 'v'])
        elif len(tree.focus()) != 0 and 'sceen' in tree.focus() and mousetype == 1:
            if distype == 0:
                boo = askyesno("提醒", "确定重新全部标准点拍照？")
                if boo != True:
                    return
                restore = ''
            elif distype == 1:
                boo = askyesno("提醒", "确定重新全部网格点拍照？")
                if boo != True:
                    return
                restore = 'Grid'
            else:
                boo = askyesno("提醒", "确定重新全部还原点拍照？")
                if boo != True:
                    return
                restore = 'New'
            t = threading.Thread(target=seriesph, args=(distype,))
            t.setDaemon(True)
            t.start()
            # for i in tree.get_children(tree.focus()):
            #     pcIP = tree.item(i, 'values')[0][:-6]
            #     port = tree.item(i, 'values')[0][-5:]
            #     client_send = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 创建socket对象
            #     tcpsult = client_send.connect_ex((pcIP, int(port)))  # 建立客户端链接
            #     if tcpsult !=0:
            #         showtext('客户端%s：%s连接失败！'%(pcIP,port),'red')
            #         return
            #     if distype == 1:  # 网格节点
            #         msg = sendmsg2(clientweival.get(),clientheival.get(),0,0,0,radiusval.get(),restore)
            #         client_send.sendall(bytes(msg, encoding="utf-8"))
            #         data = client_send.recv(1024).decode()
            #         data = data.split('"')[3].split(',')
            #         if data != '无数据':
            #             column, row = int(data[1]), int(data[0])
            #         else:
            #             showtext('未打开三角剖分！', 'red')
            #             return
            #     else:
            #         column, row = columnval.get(), rowval.get()
            #     Pc, Pr = [], []
            #     for i in range(10):
            #         if 2 ** i >= column:
            #             break
            #         Pc.append('c%s' % (i + 1))  # 按列拍照名字(不包括总的)
            #     for i in range(10):
            #         if 2 ** i >= row:
            #             break
            #         Pr.append('r%s' % (i + 1))  # 按行拍照名字(不包括总的)
            #     photocam = ['all'] + Pc + Pr
            #     if distype != 1: #标准点，还原点
            #         photocam.extend(['u','v','white'])
            #     else: #网格点
            #         photocam.extend(['u', 'v'])
            #     while len(photocam) != 0:
            #         time.sleep(3)
    except Exception as e:
        showtext(str(e.__traceback__.tb_lineno) + str(e), 'red')
# 多客户端一起连续拍照
def seriesph(distype):
    global photocam, pcIP, port, client_send,tcpsult,restore
    try:
        for i in tree.get_children(tree.focus()):
            pcIP = tree.item(i, 'values')[0][:-6]
            port = tree.item(i, 'values')[0][-5:]
            client_send = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 创建socket对象
            tcpsult = client_send.connect_ex((pcIP, int(port)))  # 建立客户端链接
            if tcpsult != 0:
                showtext('客户端%s：%s连接失败！' % (pcIP, port), 'red')
                return
            if distype == 1:  # 网格节点
                msg = sendmsg2(clientweival.get(), clientheival.get(), 0, 0, 0, radiusval.get(), restore)
                client_send.sendall(bytes(msg, encoding="utf-8"))
                data = client_send.recv(1024).decode()
                data = data.split('"')[3].split(',')
                if data != '无数据':
                    column, row = int(data[1]), int(data[0])
                else:
                    showtext('未打开三角剖分！', 'red')
                    return
            else:
                column, row = columnval.get(), rowval.get()
            Pc, Pr = [], []
            for i in range(10):
                if 2 ** i >= column:
                    break
                Pc.append('c%s' % (i + 1))  # 按列拍照名字(不包括总的)
            for i in range(10):
                if 2 ** i >= row:
                    break
                Pr.append('r%s' % (i + 1))  # 按行拍照名字(不包括总的)
            photocam = ['all'] + Pc + Pr
            if distype != 1:  # 标准点，还原点
                photocam.extend(['u', 'v', 'white'])
            else:  # 网格点
                photocam.extend(['u', 'v'])
            while len(photocam) != 0:
                time.sleep(3)
    except Exception as e:
        showtext(str(e.__traceback__.tb_lineno)+str(e), 'red')
# 精细单独拍照
def finecam():
    global photocam, pcIP, port, client_send,tcpsult,restore
    if len(tree.focus()) != 0 and 'sceen' not in tree.focus() and mousetype == 1:
        restore = 'Grid'
        pcIP = tree.item(tree.focus(), 'values')[0][:-6]
        port = tree.item(tree.focus(), 'values')[0][-5:]
        client_send = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 创建socket对象
        tcpsult = client_send.connect_ex((pcIP, int(port)))  # 建立客户端链接
        if tcpsult != 0:
            showtext('客户端%s：%s连接失败！' % (pcIP, port), 'red')
            return
        photocam = ['u1','v1']
# 场景客户端菜单栏
def conPc(event):
    menu = Menu(tree, tearoff=False)  # 创建一个菜单
    if 'sceen' in tree.focus():
        menu.add_command(label='添加客户端', command=lambda: addclientwin(event))
    else:
        menu.add_command(label='删除客户端', command=delclient)
    menu.post(event.x_root, event.y_root)
# 添加场景窗口
def addsceenwin():

    def addsceen():
        p = tree.get_children()
        pname = [tree.item(i, 'values')[0] for i in p]
        if projectval.get() not in pname and len(projectval.get()) > 0 and projectval.get().isspace() is False:
            tree.insert('', len(p), 'sceen%s' % (len(p)), values=projectval.get(), text=projectval.get(), open=True)
            add2txt2['text'] = '添加成功！'
        elif projectval.get() in pname:
            add2txt2['text'] = '场景已存在！'
        elif len(projectval.get()) == 0 or projectval.get().isspace():
            add2txt2['text'] = '场景名不能为空！'
    addtop2 = Toplevel()
    addtop2.geometry("300x100")
    addtop2.title("添加场景")
    add2txt1 = Label(addtop2,text='场景名称',font=14)
    add2txt1.place(relwidt=0.3, relheight=0.2, relx=0.05, rely=0.3)
    add2entry = Entry(addtop2,textvariable=projectval)
    add2entry.place(relwidt=0.6, relheight=0.2, relx=0.35, rely=0.3)
    add2bt = Button(addtop2,text="添加",command=addsceen)
    add2bt.place(relwidt=0.3,relheight=0.3,relx=0.25,rely=0.6)
    add2txt2 = Label(addtop2, text='', font=14)
    add2txt2.place(relwidt=0.45, relheight=0.3, relx=0.55, rely=0.6)
# 双击客户端
def openclient(event):
    if os.path.exists(pic_path) and 'sceen' not in tree.focus():
        os.startfile(pic_path)
# 添加客户端窗口
def addclientwin(event):
    def addclient():
        cname = [tree.item(i, 'values')[0] for j in tree.get_children() for i in tree.get_children(j)]
        if isIP(addipval.get()) and portval.get() in ports and addipval.get()+':'+str(portval.get()) not in cname:
            tree.insert(tree.focus(), END, text=addipval.get() + '：' + str(portval.get()),
                    values=addipval.get() + ':' + str(portval.get()))
            addtxt3['text'] = '添加成功！'
            # mkdir(paths + r'\%s\%s_%s' % (tree.item(tree.focus(), 'values')[0], addipval.get(),portval.get()))
        if addipval.get()+':'+str(portval.get()) in cname:
            addtxt3['text'] = '客户端已存在！'
        if isIP(addipval.get()) is False:
            addtxt3['text'] = 'IP格式错误！'
        if portval.get() not in ports:
            addtxt3['text'] = '端口错误！'
    ports = [10232,10233,10234,10235,10236,10237,10238,10239]
    addtop = Toplevel()
    addtop.geometry("300x100")
    addtop.title("添加客户端")
    addtxt1 = Label(addtop,text='IP',font=14)
    addtxt1.place(relwidt=0.1, relheight=0.2, relx=0.05, rely=0.2)
    addentry = Entry(addtop,textvariable=addipval)
    addentry.place(relwidt=0.4, relheight=0.2, relx=0.15, rely=0.2)
    addtxt2 = Label(addtop, text='端口', font=14)
    addtxt2.place(relwidt=0.2, relheight=0.2, relx=0.55, rely=0.2)
    addcob = tk.Combobox(addtop, values=ports, textvariable=portval)
    addcob.place(relwidt=0.2, relheight=0.2, relx=0.75, rely=0.2)
    addbt = Button(addtop,text="添加",command=addclient)
    addbt.place(relwidt=0.3,relheight=0.3,relx=0.3,rely=0.6)
    addtxt3 = Label(addtop, text='', font=14)
    addtxt3.place(relwidt=0.4, relheight=0.3, relx=0.6, rely=0.6)
# 删除客户端
def delclient():
    if 'sceen' not in tree.focus():
        tree.delete(tree.focus())
# 删除场景
def delsceen():
    if 'sceen' in tree.focus():
        tree.delete(tree.focus())
# 选择电脑ip
def selectPCip(a):
    global pic_path,projectname,pcIP,port
    if mousetype != 1.5: #不是相机拍照模式
        if 'sceen' not in tree.focus():
            pcIP = tree.item(tree.focus(),'values')[0][:-6]
            port = tree.item(tree.focus(),'values')[0][-5:]
            projectname = tree.item(tree.parent(tree.focus()))['text']
            pic_path = paths + r'\%s\%s_%s' % (projectname, pcIP, port)
        elif 'sceen' in tree.focus():
            projectname = tree.item(tree.focus())['text']
            pic_path = paths + r'\%s' % (projectname)
# 判断是否是ip地址
def isIP(str):
    p = re.compile('^((25[0-5]|2[0-4]\d|[01]?\d\d?)\.){3}(25[0-5]|2[0-4]\d|[01]?\d\d?)$')
    if p.match(str):
        return True
    else:
        return False
# ip排序
def IpSort(a):
    b = []
    for i in sorted([j[0] for j in a], key=socket.inet_aton):
        c = []
        for k in a:
            if i == k[0] and k[1] not in c:
                c.append(k[1])
        for x in sorted(c, key=socket.inet_aton):
            for y in a:
                if x == y[1] and x not in [z[1] for z in b]:
                    b.append(y)
    return b
# ip排序
def ipsort(a):
    b = []
    for i in sorted(a, key=socket.inet_aton):
        b.append(i)
    return b
# 生成随机颜色
def randcolor():
    a, b, c = 0, 0, 0
    while a == b == c:
        a = random.randint(0,255)
        b = random.randint(0,255)
        c = random.randint(0,255)
    return (a,b,c)
# 打开目录
def opendirectory():
    if os.path.exists(pic_path):
        os.startfile(pic_path)
# 选择目录
def selectdirectory():
    global pic_path
    path = askdirectory(title='选择图片文件夹')
    if os.path.exists(path):
        pic_path = path
# 圆心识别
def circle_discern(path, picname, threshold, local_division, circle_area):
    global ret_u,ret_v,ret_th,all_broken
    try:
        im = readimg(path + r'\%s.png' % (picname), 0)  # 原始图片
        mask = readimg(path + r'\mask.png', 0)  # 遮罩
        img = cv2.bitwise_and(im, mask)  #贴合模板(按位与)
        img = cv2.medianBlur(img, 3)  # 中值滤波
        h, w = im.shape[:2]
        mask0 = np.zeros((h + 2, w + 2), np.uint8)
        mask1,mask2,mask3,mask4 = mask0.copy(),mask0.copy(),mask0.copy(),mask0.copy()
        mask1[1:h + 1, 1:w + 1] = mask
        mask3[1:h + 1, 1:w + 1] = mask
        if picname == 'all': saveimg(img,path + r'\0Gray.png')
        if local_division == 0:  # 全局分割
            if picname == 'all':
                ret_th, binary = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)  # 全局阈值分割(OTSU二值化)
            else:
                ret_t, binary = cv2.threshold(img, ret_th, 255, cv2.THRESH_BINARY)
            # if ret_th < 10:
            #     local_division = 1
            #     binary = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, threshold,0)  # 局部分割
        else: # 局部分割
            binary = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, threshold, 0)  # 局部分割
            im_floodfill = binary.copy()
            cv2.floodFill(im_floodfill, mask0, (0, 0), 255)  # 漫水填充
            im_floodfill_xor = cv2.bitwise_xor(binary, im_floodfill) #按位异或
            binary = cv2.bitwise_not(im_floodfill_xor) # 按位非
        if picname == 'all': saveimg(binary, path + r'\1Binar.png')
        im_floodfill = binary.copy()
        cv2.floodFill(im_floodfill, mask1, (0, 0), 255)  # 漫水填充
        if picname == 'all': saveimg(im_floodfill,path + r'\2Floodfill.png')
        im_floodfill_inv = cv2.bitwise_not(im_floodfill)  # 按位非
        cv2.floodFill(im_floodfill_inv, mask2, (0, 0), 255)  # 漫水填充
        im_out = cv2.bitwise_not(im_floodfill_inv)  # 按位非
        if picname == 'all': saveimg(im_out,path + r'\3Floodfill.png')
        contours, hierarchy = cv2.findContours(im_out, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        min_contours = [i for i in contours if cv2.contourArea(i) < circle_area]  # 剔除小面积轮廓
        contours = [i for i in contours if cv2.contourArea(i) >= circle_area]  # 保留的轮廓
        if len(min_contours) > 0:
            for i in min_contours:
                cv2.fillPoly(im_out, [i], 0)  # 小面积轮廓涂黑
        ret, labels, stats, centroids = cv2.connectedComponentsWithStats(im_out, connectivity=8, ltype=None)  # 找圆质心
        # uv1 = [np.argwhere(labels == i) for i in range(1,len(centroids))]
        res = np.int0(np.round(centroids[1:]))
        ims = cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)
        for i in contours:
            cv2.polylines(ims, [i], True, randcolor(), 2) #画完整圆轮廓
        ims[res[:, 1], res[:, 0]] = [0, 0, 255]  # 标记完整圆质心
        if local_division == 0: # 全局分割
        # if True:
        #     if picname == 'all':
        #         all_broken = cv2.bitwise_and(binary, im_floodfill_inv)  # 残缺圆轮廓
        #         im_broken = all_broken.copy()
        #     else:
        #         im_broken = cv2.bitwise_and(all_broken,binary)
        #         im_broken = cv2.bitwise_and(im_broken,im_floodfill_inv)
            im_broken = cv2.bitwise_and(im_floodfill_inv, binary) # 残缺圆轮廓
            # saveimg(cv2.bitwise_and(im_broken, im_floodfill_inv), path + r'\3binary%s.png'%(picname))
            im_u = readimg(path + r'\u.png', 0)  # u
            im_v = readimg(path + r'\v.png', 0)  # v
            im_u, im_v = cv2.medianBlur(im_u, 5), cv2.medianBlur(im_v, 5)
            im_u, im_v = cv2.bitwise_and(im_u, im_broken), cv2.bitwise_and(im_v, im_broken)
            if picname == 'all':
                ret_u, im_u = cv2.threshold(im_u, 0, 127, cv2.THRESH_BINARY + cv2.THRESH_OTSU)  # 全局阈值分割(OTSU二值化)
                ret_v, im_v = cv2.threshold(im_v, 0, 127, cv2.THRESH_BINARY + cv2.THRESH_OTSU)  # 全局阈值分割(OTSU二值化)
            else:
                ret_u, im_u = cv2.threshold(im_u, ret_u, 127, cv2.THRESH_BINARY)
                ret_v, im_v = cv2.threshold(im_v, ret_v, 127, cv2.THRESH_BINARY)
            add_uv = im_u + im_v
            retval, add_im = cv2.threshold(add_uv, 200, 255, cv2.THRESH_BINARY)
            #去除边界贴边残缺轮廓
            cv2.floodFill(add_im, mask3, (0, 0), 255)  # 漫水填充
            add_im_inv = cv2.bitwise_not(add_im)  # 按位非
            cv2.floodFill(add_im_inv, mask4, (0, 0), 255)  # 漫水填充
            add_im = cv2.bitwise_not(add_im_inv)  # 按位非
            contours_broken, hierarchy = cv2.findContours(add_im, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            min_con = [i for i in contours_broken if cv2.contourArea(i) < 2]  # 消除面积小于2的残缺轮廓
            if len(min_con) > 0:
                for i in min_con:
                    cv2.fillPoly(add_im, [i], 0) #小面积轮廓涂黑
            green = np.argwhere(add_im == 255)
            dst = np.uint8(np.float32(add_im))
            ret, labels, stats, centroids_broken = cv2.connectedComponentsWithStats(dst, connectivity=8, ltype=None)  # 找残缺圆质心
            # uv1.extend([np.argwhere(labels == i) for i in range(1, len(centroids_broken))])
            # ret, labels, stats, broken = cv2.connectedComponentsWithStats(dst, connectivity=8, ltype=None)  # 找残缺圆质心
            # centroids_broken = np.hstack((stats[1:], broken[1:]))
            # centroids_broken = centroids_broken[np.where(centroids_broken[:, 4] > 4)][:, 5:] # 消除面积小于5的残缺轮廓
            res_broken = np.int0(np.round(centroids_broken[1:]))
            add_uv = cv2.cvtColor(cv2.bitwise_or(cv2.bitwise_and(im, im_broken), add_uv), cv2.COLOR_GRAY2BGR)
            add_uv[green[:, 0], green[:, 1]] = [0, 255, 0]
            add_uv[res_broken[:, 1], res_broken[:, 0]] = [0, 0, 255]
            # if picname == 'all': saveimg(add_uv,path + r'\4Hole.png')
            if picname == 'all':
                saveimg(cv2.bitwise_or(cv2.cvtColor(im_out, cv2.COLOR_GRAY2BGR),add_uv),path + r'\4Hole.png')
            # saveimg(cv2.bitwise_or(cv2.cvtColor(im_out, cv2.COLOR_GRAY2BGR),add_uv),path + r'\4Hole%s.png'%(picname))
            ims[res_broken[:, 1], res_broken[:, 0]] = [0, 0, 255] #标记残缺圆质心
            cr = np.vstack((centroids_broken[1:], centroids[1:]))  # 残缺质心和圆质心合并
        else:
            cr = centroids[1:]  #完整圆质心
        if picname == 'all':
            saveimg(ims,path + r'\5Contours.png')
            showtext('最小圆面积:%s(%s)' % (min([cv2.contourArea(i) for i in contours]), os.path.basename(os.path.dirname(path)).split('.')[-1]), 'black')
        n = len(contours)
        uv = np.zeros((n, 2))
        if picname == 'all':
            for i in range(n):
                (u, v), (MA, ma), angle = cv2.fitEllipse(contours[i])
                uv[i] = u, v
        # b = [[] for i in range(ret)]
        # a = []
        # a_app = a.append
        # for i in contours: #保留的轮廓
        #     for j in reversed(range(1,ret)):
        #         if j in a:
        #             continue
        #         if labels[i[0][0][1],i[0][0][0]] == j:
        #             a_app(j)
        #             b[j-1] = i
        #             break
        return cv2.cvtColor(im, cv2.COLOR_GRAY2BGR), cr, contours, uv
    except Exception as e:
        showtext(str(e.__traceback__.tb_lineno)+str(e), 'red')
# 圆心识别,distype:0是标准点,1是网格点,2是还原点 dome:0是球幕,1是环幕,2是平幕
def discern(pi_path,distype,dome):
    bname = os.path.basename(pi_path)
    try:
        if 'cam' not in bname:
            if distype == 0:  # 标准点
                pi_path = pi_path + r'\cam%s' % (vali.get())
            elif distype == 1:  # 网格点
                pi_path = pi_path + r'\Gridcam%s' % (vali.get())
            else:  # 还原点
                pi_path = pi_path + r'\Newcam%s' % (vali.get())
        # if distype == 2 and 'New' not in os.path.split(pi_path)[-1]:  # 0是识别原始图片，1是识别还原位置图片
        #     pi_path = os.path.dirname(pi_path)+r'/New%s'%(os.path.split(pi_path)[-1])
        # elif distype == 1 and 'Grid' not in os.path.split(pi_path)[-1]:  # 1是识别识别网格节点图片
        #     pi_path = os.path.dirname(pi_path)+r'/Grid%s'%(os.path.split(pi_path)[-1])
        if os.path.exists(os.path.dirname(os.path.dirname(pi_path)) + r'\mask%s.png' % (pi_path[-1])) and not os.path.exists(pi_path + r'\mask.png'):
            shutil.copyfile(os.path.dirname(os.path.dirname(pi_path)) + r'\mask%s.png' % (pi_path[-1]),pi_path + r'\mask.png')
        elif os.path.exists(os.path.dirname(pi_path) + r'\cam%s\mask.png' % (pi_path[-1])) and not os.path.exists(pi_path + r'\mask.png'):
            shutil.copyfile(os.path.dirname(pi_path) + r'\cam%s\mask.png' % (pi_path[-1]), pi_path + r'\mask.png')
        clientname = os.path.basename(os.path.dirname(pi_path)).split('.')[-1]
        showtext('开始识别:%s_%s' % (clientname, os.path.basename(pi_path)), 'blue')
        local_th = local_thval.get()  # 局部阈值分割区域大小（3~255的奇数）,值越大，白色越多，越容易黏在一起
        local_division = vali2.get()  # 0是不使用局部分割，1是使用
        area = areaval.get()  # 最小识别圆面积
        clientwei = clientweival.get()  # 客户端分辨率宽
        clienthei = clientheival.get()  # 客户端分辨率高
        if distype == 1:  # 网格节点
            row, column = gridrow.get(), gridcolumn.get()  # 网格节点行列
        else:
            row, column = rowval.get(), columnval.get()  # 圆圈行数,列数
        # co_x = list(map(round, np.linspace(0, clientwei, column + 2).tolist()))  # 横坐标
        # co_y = list(map(round, np.linspace(0, clienthei, row + 2).tolist()))  # 纵坐标
        co_x = list(map(round, np.linspace(0 - clientwei / column / 2, clientwei + clientwei / column / 2,column + 2).tolist()))  # 横坐标
        co_y = list(map(round, np.linspace(0 - clienthei / row / 2, clienthei + clienthei / row / 2, row + 2).tolist()))  # 纵坐标
        co_x[-1], co_y[-1] = co_x[-1] - 1, co_y[-1] - 1
        Pc, Pr = ['all'], ['all']
        for i in range(10):
            if 2 ** i >= column:
                break
            Pc.append('c%s' % (i + 1))  # 按列拍照名字(不包括总的)
        for i in range(10):
            if 2 ** i >= row:
                break
            Pr.append('r%s' % (i + 1))  # 按行拍照名字(不包括总的)
        Pa = Pc + Pr[1:]
        gary = [[], [], []]
        g0_app, g1_app, g2_app = gary[0].append, gary[1].append, gary[2].append
        for r in range(row):
            for c in range(column):
                g0_app(r * 100 + c)
                g1_app(round(co_x[1:-1][c]))
                g2_app(round(co_y[1:-1][r]))
        garys = pd.DataFrame(gary, index=['code', 'x', 'y']).T
        # 每张照片按列需要识别到的列序号
        columns = [[n for n in range(column) if n % (2**(i+1)) in [j + 2**(i-1) for j in range(0,2**i)]] if i != 0 else [n for n in range(column)]for i in range(len(Pc))]
        # 每张照片按行需要识别到的行序号
        rows = [[n for n in range(row) if n%(2**(i+1)) in [j + 2**(i-1) for j in range(0,2**i)]] if i != 0 else [n for n in range(row)] for i in range(len(Pr))]
        # 每列被哪几次拍照的序号
        list_c = [[pc for pc in range(len(Pc)) if c in columns[pc]] for c in range(column)]
        # 每行被哪几次拍照的序号
        list_r = [[pr for pr in range(len(Pr)) if r in rows[pr]] for r in range(row)]
        crs = (columns + rows[1:])
        discern_c, discern_r, all_point = [], [], []  # 保存按列,行识别到的点的表,保存all的图像，轮廓集，圆心集，椭圆集
        c_app, r_app = discern_c.append, discern_r.append
        for p in range(len(Pa)):
            im, cr, contours,uv = circle_discern(pi_path, Pa[p], local_th, local_division, area)
            a = pd.DataFrame(data=cr, columns=['u', 'v'])
            if Pa[p] == 'all':
                all_point = [im, cr, contours,uv]
                a.insert(loc=2, column='columns', value=str(crs[p]))
                discern_c.insert(0, a)
                b = pd.DataFrame(data=cr, columns=['u', 'v'])
                b.insert(loc=2, column='rows', value=str(crs[p]))
                discern_r.insert(0, b)
            elif 'c' in Pa[p]:
                a.insert(loc=2, column='columns', value=str(crs[p]))
                c_app(a)
            elif 'r' in Pa[p]:
                a.insert(loc=2, column='rows', value=str(crs[p]))
                r_app(a)
        for i in range(len(Pc)):
            for j, k, n in zip(discern_c[i]['u'], discern_c[i]['v'], range(len(discern_c[i]))):
                z = []
                z_app = z.append
                for a in range(len(Pc)):
                    for b, c in zip(discern_c[a]['u'], discern_c[a]['v']):
                        if abs(j - b) + abs(k - c) < 3:
                            z_app(a)
                            break
                    for v, val in enumerate(list_c):
                        if z == val:
                            discern_c[i].iat[n, 2] = v
        for i in range(len(Pr)):
            for j, k, n in zip(discern_r[i]['u'], discern_r[i]['v'], range(len(discern_r[i]))):
                z = []
                z_app = z.append
                for a in range(len(Pr)):
                    for b, c in zip(discern_r[a]['u'], discern_r[a]['v']):
                        if abs(j - b) + abs(k - c) < 3:
                            z_app(a)
                            break
                    for v, val in enumerate(list_r):
                        if z == val:
                            discern_r[i].iat[n, 2] = v
        x, y = discern_c[0], discern_r[0]
        points = discern_c[0].copy()
        points.insert(loc=3, column='rows', value=0)
        points.insert(loc=4, column='code', value=0)
        p = []
        p_app = p.append
        for i, j, k, in zip(x['u'], x['v'], range(len(x))):
            for a, b, c in zip(y['u'], y['v'], range(len(y))):
                if c in p:
                    continue
                if abs(i - a) + abs(j - b) < 5:
                    points.iat[k, 3] = y['rows'].iloc[c]
                    points.iat[k, 4] = y['rows'].iloc[c] * 100 + x['columns'].iloc[k]
                    p_app(c)
                    break
        if distype == 1:  # 网格节点
            column, row = column - 2, row - 2
        else:  # 不是网格节点
            points['rows'], points['columns'] = points['rows'] + 1, points['columns'] + 1
            points['code'] = points['code'].map(lambda x: (int(('000' + str(x))[-4:-2]) + 1) * 100 + int(('000' + str(x))[-2:]) + 1)
            garys['code'] = garys['code'].map(lambda x: (int(('000' + str(x))[-4:-2]) + 1) * 100 + int(('000' + str(x))[-2:]) + 1)
            for r in range(row + 2):
                for c in [0, column + 1]:
                    s = pd.Series([r * 100 + c, co_x[c], co_y[r]], index=['code', 'x', 'y'])
                    garys = garys.append(s, ignore_index=True)
            for r in [0, row + 1]:
                for c in range(1, column + 1):
                    s = pd.Series([r * 100 + c, co_x[c], co_y[r]], index=['code', 'x', 'y'])
                    garys = garys.append(s, ignore_index=True)
            garys = garys.sort_values(by='code')
        pp = points.copy()
        showtext('识别点数:%s(%s)' % (len(pp), clientname), 'black')
        points.drop_duplicates(subset=['code'], keep=False, inplace=True) #删除重复的点
        ps = pd.concat([points, pp])
        ps = ps.drop_duplicates(subset=['u', 'v', 'code'], keep=False)  # 重复的点
        ps = ps[~ps['rows'].isin(['['])] #去掉未匹配的点
        ps, pss, psx = ps.sort_values(by='code'), [], []
        calibration = pd.merge(garys, points.loc[:, ['code', 'u', 'v']], how='outer', on='code')
        # print(len(calibration))
        calx = np.array(calibration['u']).reshape((row + 2, column + 2))
        caly = np.array(calibration['v']).reshape((row + 2, column + 2))
        if len(points) < 10: #识别点数小于10
            filename = pi_path + r'\cam%s.csv' % (pi_path[-1])
            calibration.to_csv(defile(filename), mode='a', header=None)
            showtext('总点数:%s(%s)' % (len(calibration), clientname), 'black')
            return
        if len(ps) > 0:  # 错点选一
            # print(list(set(list(ps['code']))))
            for i in list(set(list(ps['code']))):
                me = []
                for j in range(len(ps)):
                    ne, r, c = [], ps['rows'].iloc[j], ps['columns'].iloc[j]
                    if i == ps['code'].iloc[j]:
                        if -1 < c - 1:
                            if ~np.isnan(calx[r, c - 1]):
                                ne.append([calx[r, c - 1], caly[r, c - 1]])
                        if column + 2 > c + 1:
                            if ~np.isnan(calx[r, c + 1]):
                                ne.append([calx[r, c + 1], caly[r, c + 1]])
                        if -1 < r - 1:
                            if ~np.isnan(calx[r - 1, c]):
                                ne.append([calx[r - 1, c], caly[r - 1, c]])
                        if row + 2 > r + 1:
                            if ~np.isnan(calx[r + 1, c]):
                                ne.append([calx[r + 1, c], caly[r + 1, c]])
                    if len(ne) > 0:
                        me.append([sum([(k[0] - ps['u'].iloc[j]) ** 2 + (k[1] - ps['v'].iloc[j]) ** 2 for k in ne])[
                                       0] / len(ne), j])
                me.sort(key=lambda x: x[0])
                if len(me) > 0:
                    ro, co = ps['rows'].iloc[me[0][1]], ps['columns'].iloc[me[0][1]]
                    calx[ro, co], caly[ro, co] = ps['u'].iloc[me[0][1]], ps['v'].iloc[me[0][1]]
                    s = pd.Series([ps['u'].iloc[me[0][1]], ps['v'].iloc[me[0][1]], co, ro, i],
                                  index=['u', 'v', 'columns', 'rows', 'code'])
                    points = points.append(s, ignore_index=True)
                    pss.append(i)  # 正确的点
                    for j in range(1, len(me)):
                        psx.append([ps['u'].iloc[me[j][1]], ps['v'].iloc[me[j][1]]])  # 错误的点
            points[['columns', 'rows', 'code']] = points[['columns', 'rows', 'code']].astype(int)
        if distype == 1:  # 网格节点
            basic = pd.read_csv(os.path.dirname(pi_path) + r'\basic.txt', names=['c', 'r', 'x', 'y'], sep=' ')
            calibration[['x', 'y']] = basic[['c', 'r']]
            flys = []
            if dome == 0:  # 球幕
                calx, caly, flys = killfly(calx, caly, row, column)
        else:  # 不是网格节点
            calx, caly, flys = killfly(calx, caly, row, column)
        calibration['u'] = [i for j in calx for i in j]
        calibration['v'] = [i for j in caly for i in j]
        interpoint = str(len(calx[~np.isnan(calx)]))
        showtext('保存点数:%s(%s)' % (interpoint, clientname), 'black')
        interpoint = interpoint + r' %s' % (len(calibration))
        txtfile = open(pi_path + r'\cam%s.txt' % (pi_path[-1]), 'w')
        txtfile.write(interpoint)
        # for i in calibration0['code']:
        #     txtfile.write('\n')
        #     txtfile.write(str(i))
        txtfile.close()
        im = all_point[0].copy()
        for i in all_point[2]:
            cv2.polylines(im, [i], True, randcolor(), 1)
            # cv2.ellipse(im, tuple(all_point[2][i]), randcolor(), 1)  # 画椭圆
        if len(pss) > 0:
            fpoints = points[points['code'].isin([i for i in pss if i not in flys])]
            for i in range(len(fpoints)):
                for j in range(len(all_point[3])):
                    if abs(all_point[3][j][0] - fpoints['u'].iloc[i]) + abs(
                            all_point[3][j][1] - fpoints['v'].iloc[i]) < 5:
                        # cv2.ellipse(im, tuple(all_point[2][j]), (0, 255, 0), 3)  # 画正确的点
                        cv2.polylines(im, [all_point[2][j]], True, (0, 255, 0), 3) # 画正确的点
            # print(psx)
            # print(len(all_point[2]))
            for i in psx: # 错误的点
                for j in range(len(all_point[3])):
                    if abs(all_point[3][j][0] - i[0]) + abs(all_point[3][j][1] - i[1]) < 5:
                        # cv2.ellipse(im, tuple(all_point[2][j]), (0, 255, 0), -1)  # 画错误的点
                        # cv2.polylines(im, [all_point[2][j]], True, (0, 255, 0), -1)  # 画错误的点
                        cv2.fillPoly(im, [all_point[2][j]], (0, 255, 0)) # 画错误的点
        if len(flys) > 0:
            fpoints = points[points['code'].isin(flys)]
            for i in range(len(fpoints)): #飞点
                for j in range(len(all_point[3])):
                    if abs(all_point[3][j][0] - fpoints['u'].iloc[i]) + abs(all_point[3][j][1] - fpoints['v'].iloc[i]) < 5:
                        # cv2.ellipse(im, tuple(all_point[2][j]), (114, 88, 199), -1)  # 画飞点
                        # cv2.polylines(im, [all_point[2][j]], True, (114, 88, 199), -1)  # 画飞点
                        cv2.fillPoly(im, [all_point[2][j]], (114, 88, 199))  # 画飞点
        # for i in range(len(points)):
        #     cc, rr = int(round(points['u'].iloc[i])), int(round(points['v'].iloc[i]))  # 四舍五入
        #     cv2.line(im, (cc - 5, rr), (cc + 5, rr), (0, 0, 255), 1)  # 原图上画十字光标
        #     cv2.line(im, (cc, rr - 5), (cc, rr + 5), (0, 0, 255), 1)
        #     cv2.putText(im, str(points['rows'].iloc[i]) + ',' + str(points['columns'].iloc[i]), (cc, rr - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1)
        # saveimg(im,pi_path + r'\6Discern.png')
        # mask = cv2.bitwise_not(readimg(pi_path + r'\mask.png', 0))
        # mask[mask == 255] = 25
        mask = readimg(pi_path + r'\mask.png', 0)
        draw = all_point[0].copy()
        pich, picw = all_point[0].shape[:2] #图像高宽
        m = np.zeros((pich + 2, picw + 2), np.uint8)
        contours1, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for i in contours1:  # 所有轮廓
            cv2.polylines(draw, [i], True, (0, 0, 255), 2)
            cv2.polylines(im, [i], True, (0, 0, 255), 2)
        cv2.floodFill(mask, m, (0, 0), 255)  # 漫水填充
        contours2, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        area = [[cv2.contourArea(i), i] for i in contours2]
        area.sort(key=lambda x: x[0], reverse=True)
        area = area[1:]  # 遮罩的轮廓
        for i in area:
            cv2.polylines(draw, [i[1]], True, (0, 255, 0), 2)
            cv2.polylines(im, [i[1]], True, (0, 255, 0), 2)
        # draw2 = draw.copy()
        if len(all_point[1]) > 0:
            # ims = drawline(draw, calibration, row, column)
            ims = drawline(im, calibration, row, column)
            saveimg(ims, pi_path + r'\6Discern.png')
            # saveimg(ims,pi_path + r'\6LineCircle.png')
        calx = np.array(calibration['u']).reshape((row + 2, column + 2))
        caly = np.array(calibration['v']).reshape((row + 2, column + 2))
        if vali4.get() == 1:  # 插值
            # 内部插值
            calibration['u'] = [i for j in calx for i in j]
            calibration['v'] = [i for j in caly for i in j]

            calx = np.array(calibration['u']).reshape((row + 2, column + 2))
            g0 = pointsort0(calx)  # 内部满点
            if distype != 1 or dome == 0:  # 不是网格节点或者是球幕
                g1 = pointsort2(calx)  # 外一圈角点
                for i in g1:
                    if i not in g0:
                        g0.append(i)
            calibration1 = calibration.dropna(axis=0, subset=['u', 'x'])
            calibration2 = calibration[calibration['code'].isin([i[0] * 100 + i[1] for i in g0])]
            calibration3 = calibration[~calibration['code'].isin([i[0] * 100 + i[1] for i in g0])]
            calibration2 = interp2D(calibration1, calibration2, 'x', 'y', 'u', 'v', 'quintic')  # 二维插值
            calibration = calibration3.append(calibration2).sort_values(by='code')
            # calx = np.array(calibration['u']).reshape((row + 2, column + 2))
            # g0 = pointsort1(calx)  # 内部满点
            # calibration1 = calibration.dropna(axis=0, subset=['u', 'x'])
            # calibration2 = calibration[calibration['code'].isin([i[0] * 100 + i[1] for i in g0])]
            # calibration3 = calibration[~calibration['code'].isin([i[0] * 100 + i[1] for i in g0])]
            # calibration2 = interp2D(calibration1, calibration2, 'x', 'y', 'u', 'v', 'quintic')  # 二维插值
            # calibration = calibration3.append(calibration2).sort_values(by='code')

            calx = np.array(calibration['u']).reshape((row + 2, column + 2))
            caly = np.array(calibration['v']).reshape((row + 2, column + 2))

            if distype != 1 or dome == 0:  # 不是网格节点或者是球幕
                calx, caly, flys = killfly(calx, caly, row, column)

            if local_division == 1: #局部分割
                calx, caly = interpolation(calx, caly, column, row, 2, distype, dome)  # 外部插两圈
            else:
                calx, caly = interpolation(calx, caly, column, row, 1, distype, dome)  # 外部插一圈
            calibration['u'] = [i for j in calx for i in j]
            calibration['v'] = [i for j in caly for i in j]
            calibration = calibration.reset_index(drop=True)
            showtext('插值后保存点数:%s(%s)' % (len(calx[~np.isnan(calx)]), clientname), 'black')
            cl = calibration.copy()
            w0,h0,w1,h1 = 0,0,0,0
            if min(cl['u'][cl['u'].notna()]) < 0:
                w0 = round(abs(min(cl['u'][cl['u'].notna()])))+30
            if max(cl['u'][cl['u'].notna()]) > picw:
                w1 = round(max(cl['u'][cl['u'].notna()]) - picw)+30
            if min(cl['v'][cl['v'].notna()]) < 0:
                h0 = round(abs(min(cl['v'][cl['v'].notna()])))+30
            if max(cl['v'][cl['v'].notna()]) > pich:
                h1 = round(max(cl['v'][cl['v'].notna()]) - pich)+30
            interpicw, interpich= picw + w0 +w1, pich + h0 +h1
            interpic = np.zeros((interpich, interpicw ,3), np.uint8)
            interpic[h0:h0+pich,w0:w0+picw] = draw
            cl['u'] = cl['u'] + w0
            cl['v'] = cl['v'] + h0

            imp = drawline(interpic, cl, row, column)
            # imp = drawline(draw2, calibration, row, column)
            saveimg(imp,pi_path + r'\7Interp.png')
        else:  # 不插值
            calibration['u'] = [i for j in calx for i in j]
            calibration['v'] = [i for j in caly for i in j]
        filename = pi_path + r'\cam%s.csv' % (pi_path[-1])
        calibration.to_csv(defile(filename), mode='a', header=None)
        showtext('总点数:%s(%s)' % (len(calibration), clientname), 'black')
        showtext('识别完成(%s)' % (clientname), 'black')
    except Exception as e:
        showtext(str(e.__traceback__.tb_lineno) + str(e) + bname,'red')
# 画线条图
def drawline(imp,calibration,row,column):
    u = []
    u_app = u.append
    for i, j, k in zip(calibration['u'], calibration['v'], calibration['code']):
        if math.isnan(i) is False:
            u_app([int(i), int(j)])
            cv2.circle(imp, (int(i), int(j)), 2, (0, 0, 255), -1)
            cv2.putText(imp, str(int(('000' + str(int(k)))[-4:-2])) + ',' + str(int(('000' + str(int(k)))[-2:])),
                        (int(i), int(j) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,255,255), 1)
        else:
            u_app([i, j])
    u = np.array(u).reshape((row + 2, column + 2, 2))
    for i in range(row + 2):
        a = list(np.where(~np.isnan(u[i]))[0])
        if not a:
            continue
        b = []
        for j in range(column + 2):
            if j in a:
                b.append(j)
            else:
                if len(b)>1:
                    cv2.polylines(imp, [u[i,b[0]:b[-1]+1].astype(int)], False, (255, 255, 0), 1)
                b = []
            if j == a[-1]:
                cv2.polylines(imp, [u[i, b[0]:b[-1] + 1].astype(int)], False, (255, 255, 0), 1)
                b = []
            elif j>a[-1]:
                break
        # cv2.polylines(imp, [u[i][~np.isnan(u[i]).any(axis=1)].astype(int)], False, (255, 255, 0), 1)
    for i in range(column + 2):
        a = list(np.where(~np.isnan(u[:,i]))[0])
        if not a:
            continue
        b = []
        for j in range(row + 2):
            if j in a:
                b.append(j)
            else:
                if len(b) > 1:
                    cv2.polylines(imp, [u[b[0]:b[-1]+1,i].astype(int)], False, (0, 255, 0), 1)
                b = []
            if j == a[-1]:
                cv2.polylines(imp, [u[b[0]:b[-1] + 1, i].astype(int)], False, (0, 255, 0), 1)
                b = []
            elif j>a[-1]:
                break
        # cv2.polylines(imp, [u[:, i][~np.isnan(u[:, i]).any(axis=1)].astype(int)], False, (0, 255, 255), 1)
    return imp
# 去除识别错误的飞点
def killfly(calx,caly,row,column):
    try:
        f = flyval.get()
        uv,au,av,fly,flys = [],[],[],[],[]
        uv_app,au_app,av_app,fly_app,flys_app = uv.append,au.append,av.append,fly.append,flys.append
        for i in range(row + 2):  # 遍历行
            for j in np.where(~np.isnan(calx[i]))[0]:
                uv_app([i, j])
        for i in range(row+2):
            u = []
            for j in uv:
                if j[0] == i:
                    u.append(j)
            if u != []:
                au_app(u)
        for i in range(column+2):
            v = []
            for j in uv:
                if j[1] == i:
                    v.append(j)
            if v != []:
                av_app(v)
        for i in au:
            aa = []
            if len(i)>3:
                for j in range(len(i)-1):
                    xx = (calx[i[j][0],i[j][1]]-calx[i[j+1][0],i[j+1][1]])/(i[j][1]-i[j+1][1])
                    yy = (caly[i[j][0],i[j][1]]-caly[i[j+1][0],i[j+1][1]])/(i[j][1]-i[j+1][1])
                    aa.append([(xx**2+yy**2)**0.5,i[j],i[j+1]])
                aaa = aa[:]
                aaa.sort(key=lambda x:x[0])
                me = np.median([i[0] for i in aaa][1:-1])
                for k in range(len(aa)):
                    # print(aa[k][0]/me)
                    if aa[k][0]/me>f or me/aa[k][0]>f:
                        if k == 0:
                            if aa[k+1][0]/me>f or me/aa[k+1][0]>f:
                                fly_app(aa[k][2])
                            else:
                                fly_app(aa[k][1])
                        else:
                            if aa[k-1][0]/me>f or me/aa[k-1][0]>f:
                                fly_app(aa[k][1])
                            else:
                                fly_app(aa[k][2])
        for i in av:
            aa = []
            if len(i)>3:
                for j in range(len(i)-1):
                    xx = (calx[i[j][0],i[j][1]]-calx[i[j+1][0],i[j+1][1]])/(i[j][0]-i[j+1][0])
                    yy = (caly[i[j][0],i[j][1]]-caly[i[j+1][0],i[j+1][1]])/(i[j][0]-i[j+1][0])
                    aa.append([(xx**2+yy**2)**0.5,i[j],i[j+1]])
                aaa = aa[:]
                aaa.sort(key=lambda x:x[0])
                me = np.median([i[0] for i in aaa][1:-1])
                for k in range(len(aa)):
                    if aa[k][0]/me>f or me/aa[k][0]>f:
                        if k == 0:
                            if aa[k+1][0]/me>f or me/aa[k+1][0]>f:
                                fly_app(aa[k][2])
                            else:
                                fly_app(aa[k][1])
                        else:
                            if aa[k-1][0]/me>f or me/aa[k-1][0]>f:
                                fly_app(aa[k][1])
                            else:
                                fly_app(aa[k][2])
        for i in fly:
            if i[0]*100+i[1] not in flys:
                flys_app(i[0]*100+i[1])
                showtext('飞点'+str(i[0])+','+str(i[1]),'red')
                calx[i[0],i[1]], caly[i[0],i[1]] = np.nan,np.nan

        # row, column = calx.shape[0], calx.shape[1]
        # calxy = np.dstack((calx, caly))
        # calc = np.zeros((row, column - 1), np.float32)  # 列间距
        # calr = np.zeros((row - 1, column), np.float32)  # 行间距
        # for i in range(row):
        #     for j in range(column - 1):
        #         calc[i, j] = segment(calxy[i, j], calxy[i, j + 1])
        # for j in range(column):
        #     for i in range(row - 1):
        #         calr[i, j] = segment(calxy[i, j], calxy[i + 1, j])
        # print(calc[5,:][~np.isnan(calc[5,:])])#.astype(int))


        return calx, caly, flys
    except Exception as e:
        showtext(str(e.__traceback__.tb_lineno) + str(e), 'red')
# 内部满点
def pointsort0(ca):
    row, column = ca.shape[0], ca.shape[1]
    g = []
    g_app = g.append
    a = [list(np.where(~np.isnan(ca[i]))[0]) for i in range(row)]  # 每一行非空列序号
    b = [list(np.where(~np.isnan(ca[:, i]))[0]) for i in range(column)]  # 每一列非空行序号
    for i in range(len(a)):
        if a[i]:
            if a[i][-1]-a[i][0]+1 != len(a[i]):
                if len(a[i])/(a[i][-1]-a[i][0]+1) >0.6: #非空点占比达到0.6
                    for j in range(a[i][0]+1,a[i][-1]):
                        if j not in a[i]:
                            g_app([i, j])
                else:
                    for j in range(a[i][0]+1,a[i][-1]):
                        if j not in a[i] and b[j]:
                            if b[j][0] < i < b[j][-1]:
                                g_app([i, j])
    return g
# 找两个圆的交点
def insec(pr1,pr2,p3):
    try:
        p1,r1,p2,r2 = pr1[0],pr1[1],pr2[0],pr2[1]
        x,y,a,b = p1[0],p1[1],p2[0],p2[1]
        d = math.sqrt((abs(a - x)) ** 2 + (abs(b - y)) ** 2)
        if d > (r1 + r2) or d < (abs(r1 - r2)):
            return [np.nan, np.nan]
        elif d == 0:
            return [np.nan, np.nan]
        else:
            A = (r1**2 - r2**2 + d**2) / (2*d)
            h = math.sqrt(r1**2 - A**2)
            x2,y2 = x + A * (a - x) / d,y + A * (b - y) / d
            x3,y3 = round(x2 - h * (b - y) / d, 2),round(y2 + h * (a - x) / d, 2)
            x4,y4 = round(x2 + h * (b - y) / d, 2),round(y2 - h * (a - x) / d, 2)
            c1,c2 = [x3, y3],[x4, y4]
            s1 = math.sqrt((abs(p3[0] - x3)) ** 2 + (abs(p3[1] - y3)) ** 2)
            s2 = math.sqrt((abs(p3[0] - x4)) ** 2 + (abs(p3[1] - y4)) ** 2)
            if s1 > s2:
                return c1
            else:
                return c2
    except Exception as e:
        showtext(str(e.__traceback__.tb_lineno) + str(e),'red')
# 外一圈点
def pointsort1(ca):
    row, column = ca.shape[0], ca.shape[1]
    b = []
    b_app = b.append
    a = [list(np.where(~np.isnan(ca[i]))[0]) for i in range(row)]
    for i in range(len(a)):
        for j in a[i]:
            if j - 1 not in a[i]:
                b_app([i, j - 1])
            if j + 1 not in a[i]:
                b_app([i, j + 1])
            if i != 0:
                if j - 1 not in a[i - 1]:
                    b_app([i - 1, j - 1])
                if j not in a[i - 1]:
                    b_app([i - 1, j])
                if j + 1 not in a[i - 1]:
                    b_app([i - 1, j + 1])
            if i != row - 1:
                if j - 1 not in a[i + 1]:
                    b_app([i + 1, j - 1])
                if j not in a[i + 1]:
                    b_app([i + 1, j])
                if j + 1 not in a[i + 1]:
                    b_app([i + 1, j + 1])
    #     k = ''
    #     for j in range(column):
    #         if j in a[i]:
    #             k = k + '1'
    #         else:
    #             k = k + '0'
    #     c_app(k)
    # for i in range(column):  # 遍历列
    #     listc = list(np.where(~np.isnan(ca[:, i]))[0])  # 非空的列序号
    #     k = ''
    #     for j in range(row + 2):
    #         if j in listc:
    #             k = k + '1'
    #         else:
    #             k = k + '0'
    #     d_app(k)
    bb = []
    for i in b:
        if i not in bb:
            if i[1] > -1 and i[1] < column:
                bb.append(i)
    return bb
# 找外一圈中的角点
def pointsort2(ca):
    row, column = ca.shape[0], ca.shape[1]
    c, d, e = [], [], []
    c_app, d_app, e_app = c.append, d.append, e.append
    a = [list(np.where(~np.isnan(ca[i]))[0]) for i in range(row)] #每一行非空列序号
    b = [list(np.where(~np.isnan(ca[:, i]))[0]) for i in range(column)] #每一列非空行序号
    for i in range(len(a)):
        if a[i]:
            c_app([i, a[i][0] - 1])
            c_app([i, a[i][-1] + 1])
    for j in range(len(b)):
        if b[j]:
            d_app([b[j][0] - 1, j])
            d_app([b[j][-1] + 1, j])
    for i in c:
        if i in d:
            if i[0] > -1 and i[0] < row and i[1] > -1 and i[1] < column:
                e.append(i)
    return e

def pointsort3(b,e,f):
    eee,fff = [],[]
    for i in b:
        ee,ff = [],[]
        for j in e:
            if j[0] == i and abs(j[1]) !=3:
                ee.append(j)
        if len(ee) != 0:
            eee.append(ee)
        for j in f:
            if j[0] == i and abs(j[1]) !=3:
                ff.append(j)
        if len(ff) != 0:
            fff.append(ff)
    return eee,fff
# 内部插值
def pointsort4(e,f,ca,cb):
    k = 0.6 #非空值在需要内部插值的行列所占比例为k，才进行插值
    for i in e:
        if len(i[2][2])<4:
            continue
        x,y = i[0][0],i[0][1]
        fx = interp1d(i[2][2],i[2][3],kind="cubic") #编辑插值函数格式
        if (abs(i[1])==1 or i[1]==3)and y>i[2][0] and y<i[2][1] and len(i[2][2])>(i[2][2][-1]-i[2][2][0])*k and np.isnan(ca[x,y]):
            ca[x,y] = fx(y)
        elif (abs(i[1])==2 or i[1]==-3)and x>i[2][0] and x<i[2][1] and len(i[2][2])>(i[2][2][-1]-i[2][2][0])*k and np.isnan(ca[x,y]):
            ca[x,y] = fx(x)
    for i in f:
        if len(i[2][2])<4:
            continue
        x,y = i[0][0],i[0][1]
        fy = interp1d(i[2][2],i[2][3],kind="cubic") #编辑插值函数格式
        if (abs(i[1])==1 or i[1]==3)and y>i[2][0] and y<i[2][1] and len(i[2][2])>(i[2][2][-1]-i[2][2][0])*k and np.isnan(cb[x,y]):
            cb[x,y] = fy(y)
        elif (abs(i[1])==2 or i[1]==-3)and x>i[2][0] and x<i[2][1] and len(i[2][2])>(i[2][2][-1]-i[2][2][0])*k and np.isnan(cb[x,y]):
            cb[x,y] = fy(x)
    return ca,cb
# 拉格朗日插值 pp:插几圈,distype:0是标准点,1是网格点,2是还原点 dome:0是球幕,1是环幕,2是平幕
def interpolation(ca,cb,column,row,pp,distype,dome):
    try:
        row, column = ca.shape[0], ca.shape[1]
        while pp != 0:
            bb = pointsort2(ca)  # 外一圈角点
            ff = pointsort1(ca) #外一圈点
            for i in bb:
                ff.remove(i)
            bb.extend(ff)
            cc = bb[:]
            ff = 0
            if distype == 1 and dome != 0: #网格节点，非球幕
            # if True:
                ee = pointsort2(ca)  # 外一圈角点
                calxy = np.dstack((ca, cb))
                calc = np.zeros((row, column - 1), np.float32)  # 列间距
                calr = np.zeros((row - 1, column), np.float32)  # 行间距
                for i in range(row):
                    for j in range(column - 1):
                        calc[i, j] = segment(calxy[i, j], calxy[i, j + 1])
                for j in range(column):
                    for i in range(row - 1):
                        calr[i, j] = segment(calxy[i, j], calxy[i + 1, j])
            while bb: #一圈插完
                if distype == 1 and dome != 0:  # 网格节点，非球幕
                # if True:
                    if ee:
                        tt = []
                        for x, y in ee:
                            s = []
                            # 左上角
                            if y > 1 and x > 0:
                                if ~np.isnan(calr[x - 1, y - 2:y]).any():  # 行间距
                                    s.append([calxy[x - 1, y], lagrange([1, 2], calr[x - 1, y - 2:y])(3)])
                            if x > 1 and y > 0:
                                if ~np.isnan(calc[x - 2:x, y - 1]).any():  # 列间距
                                    s.append([calxy[x, y - 1], lagrange([1, 2], calc[x - 2:x, y - 1])(3)])
                            if len(s) == 2:
                                [ca[x, y], cb[x, y]] = insec(s[0], s[1], calxy[x - 1, y - 1])
                                if ~np.isnan(ca[x, y]):
                                    bb.remove([x, y])
                                    tt.append([x, y])
                                continue
                            s = []
                            # 左下角
                            if y > 1 and x < row - 1:
                                if ~np.isnan(calr[x, y - 2:y]).any():
                                    s.append([calxy[x + 1, y], lagrange([1, 2], calr[x, y - 2:y])(3)])
                            if x < row - 2 and y > 0:
                                if ~np.isnan(calc[x + 1:x + 3, y - 1]).any():
                                    s.append([calxy[x, y - 1], lagrange([1, 2], calc[x + 1:x + 3, y - 1][::-1])(3)])
                            if len(s) == 2:
                                [ca[x, y], cb[x, y]] = insec(s[0], s[1], calxy[x + 1, y - 1])
                                if ~np.isnan(ca[x, y]):
                                    bb.remove([x, y])
                                    tt.append([x, y])
                                continue
                            s = []
                            # 右上角
                            if x > 1 and y < column - 1:
                                if ~np.isnan(calc[x - 2:x, y]).any():
                                    s.append([calxy[x, y + 1], lagrange([1, 2], [calc[x - 2, y], calc[x - 1, y]])(3)])
                            if y < column - 2 and x > 0:
                                if ~np.isnan(calr[x - 1, y + 1:y + 3]).any():
                                    s.append([calxy[x - 1, y], lagrange([1, 2], calr[x - 1, y + 1:y + 3][::-1])(3)])
                            if len(s) == 2:
                                [ca[x, y], cb[x, y]] = insec(s[0], s[1], calxy[x - 1, y + 1])
                                if ~np.isnan(ca[x, y]):
                                    bb.remove([x, y])
                                    tt.append([x, y])
                                continue
                            s = []
                            # 右下角
                            if x < row - 2 and y < column - 1:
                                if ~np.isnan(calc[x + 1:x + 3, y]).any():
                                    s.append([calxy[x, y + 1], lagrange([1, 2], calc[x + 1:x + 3, y][::-1])(3)])
                            if y < column - 2 and x < row - 1:
                                if ~np.isnan(calr[x, y + 1:y + 3]).any():
                                    s.append([calxy[x + 1, y], lagrange([1, 2], calr[x, y + 1:y + 3][::-1])(3)])
                            if len(s) == 2:
                                [ca[x, y], cb[x, y]] = insec(s[0], s[1], calxy[x + 1, y + 1])
                                if ~np.isnan(ca[x, y]):
                                    bb.remove([x, y])
                                    tt.append([x, y])
                        if len(ee)==len(tt): #角点全满足插值条件
                            ee = pointsort2(ca)  # 外一圈角点
                            ee = [i for i in ee if i in bb]
                            if ee:
                                calxy = np.dstack((ca, cb))
                                for i in range(row):
                                    for j in range(column - 1):
                                        calc[i, j] = segment(calxy[i, j], calxy[i, j + 1])
                                for j in range(column):
                                    for i in range(row - 1):
                                        calr[i, j] = segment(calxy[i, j], calxy[i + 1, j])
                                continue
                            else:
                                dd = bb[:]
                        else:
                            dd = [i for i in ee if i not in tt]
                    else:
                        dd = bb[:]
                else:
                    dd = bb[:]
                e, f, g = [], [], []
                e_app, f_app, g_app = e.append, f.append, g.append
                for x, y in dd:
                    s, t = [], []
                    if x > 1:
                        if ~np.isnan(ca[x - 2:x, y]).any():
                            s.append(lagrange([1, 2], ca[x - 2:x, y])(3))
                            t.append(lagrange([1, 2], cb[x - 2:x, y])(3))
                    if x < row - 2:
                        if ~np.isnan(ca[x + 1:x + 3, y]).any():
                            s.append(lagrange([1, 2], ca[x + 1:x + 3, y][::-1])(3))
                            t.append(lagrange([1, 2], cb[x + 1:x + 3, y][::-1])(3))
                    if y > 1:
                        if ~np.isnan(ca[x, y - 2:y]).any():
                            s.append(lagrange([1, 2], ca[x, y - 2:y])(3))
                            t.append(lagrange([1, 2], cb[x, y - 2:y])(3))
                    if y < column - 2:
                        if ~np.isnan(ca[x, y + 1:y + 3]).any():
                            s.append(lagrange([1, 2], ca[x, y + 1:y + 3][::-1])(3))
                            t.append(lagrange([1, 2], cb[x, y + 1:y + 3][::-1])(3))
                    if s:
                        e_app([x, y, np.mean(s)])
                        f_app([x, y, np.mean(t)])
                        g_app([x, y])
                for x, y, s in e:
                    ca[x, y] = s
                for x, y, t in f:
                    cb[x, y] = t
                for i in g:
                    bb.remove(i)
                if len(bb) == len(cc) or len(bb) == ff: #没有插点，插不进去
                    break
                ff = len(bb)
                if distype == 1 and dome != 0:  # 网格节点，非球幕
                # if True:
                    ee = pointsort2(ca)  # 外一圈角点
                    ee = [i for i in ee if i in bb]
                    if ee:
                        calxy = np.dstack((ca, cb))
                        for i in range(row):
                            for j in range(column - 1):
                                calc[i, j] = segment(calxy[i, j], calxy[i, j + 1])
                        for j in range(column):
                            for i in range(row - 1):
                                calr[i, j] = segment(calxy[i, j], calxy[i + 1, j])
            pp -= 1
            if True not in np.isnan(ca):
                break
            if distype == 1 and dome != 0: #网格节点且非球幕插满
                pp = 1
        if distype == 1 and dome == 0: #网格节点,球幕上下各四行值为空
            ca[:5],ca[-4:],cb[:5],cb[-4:] = np.NaN,np.NaN,np.NaN,np.NaN
        return ca,cb
    except Exception as e:
        showtext(str(e.__traceback__.tb_lineno) + str(e),'red')
# 二维差值
def interp2D(df1,df2,c,r,u,v,fun):
    df3 = df2.copy()
    x, y, u1, v1 = df1[c], df1[r], df1[c] - df1[u], df1[r] - df1[v]
    # x,y,u1,v1 = df1[c],df1[r],df1[c]-df1[u]-(max(df1[u])+min(df1[u]))/2,df1[r]-df1[v]-(max(df1[v])+min(df1[v]))/2
    x2, y2 = df2[c], df2[r]
    fu, fv = Rbf(x, y, u1, function=fun), Rbf(x, y, v1, function=fun)
    u2, v2 = fu(x2, y2), fv(x2, y2)  # 输入输出都是二维
    df3.loc[:, u], df3.loc[:, v] = df2.loc[:, c] - u2, df2.loc[:, r] - v2
    # df3.loc[:,u],df3.loc[:,v] = df2.loc[:,c]-u2+(max(df1[u])+min(df1[u]))/2,df2.loc[:,r]-v2+(max(df1[v])+min(df1[v]))/2
    return df3
# 三维插值
def interp3D(df1,df2,c,r,u,v):
    df3 = df2.copy()
    xy,u1,v1 = df1[[c,r]],df1[c]-df1[u],df1[r]-df1[v]
    xy2 = df2[[c,r]]
    u2 = griddata(xy, u1, xy2, method='cubic', fill_value=np.nan, rescale=False)  # 三维插值
    v2 = griddata(xy, v1, xy2, method='cubic',fill_value=np.nan, rescale=False)#三维插值
    df3.loc[:, u], df3.loc[:, v] = df2.loc[:, c] - u2, df2.loc[:, r] - v2
    return df3
# 单独插值
def interp():
    try:
        data_path = askopenfilename(title='选择需要插值的文件',filetype=[("","*.csv")])  # 打开需要插值的csv路径
        if os.path.exists(data_path):
            df = pd.read_csv(data_path, header=None)
            df.columns = ['num','code', 'x', 'y', 'u', 'v']
            row, column = int(('00' + str(int(df['code'].iloc[-1])))[-4:-2]) - 1, int(('00' + str(int(df['code'].iloc[-1])))[-2:]) - 1
            if df['u'].isnull().sum() == 0:
                showtext('点已插满，无需再插！', 'red')
                return
            df1 = df.dropna(axis=0, subset=['u', 'v'])
            xy = df1[['x', 'y']]
            u, v = df1['x'] - df1['u'], df1['y'] - df1['v']
            calx = np.array(df['u']).reshape((row + 2, column + 2))
            g = pointsort0(calx)  # 内部满点
            df2 = df[df['code'].isin([i[0] * 100 + i[1] for i in g])]
            df3 = df[~df['code'].isin([i[0] * 100 + i[1] for i in g])]
            xy2 = df2[['x', 'y']]
            u2 = griddata(xy, u, xy2, method='cubic', fill_value=np.nan, rescale=False)  # 三维插值
            v2 = griddata(xy, v, xy2, method='cubic', fill_value=np.nan, rescale=False)  # 三维插值
            df2['u'], df2['v'] = df2['x'] - u2, df2['y'] - v2
            df = df3.append(df2).sort_values(by='code')
            # 外部插一圈
            calx = np.array(df['u']).reshape((row + 2, column + 2))
            caly = np.array(df['v']).reshape((row + 2, column + 2))

            calx, caly = interpolation(calx, caly, column, row, 1,0,1)  # 插一圈
            df['u'] = [i for j in calx for i in j]
            df['v'] = [i for j in caly for i in j]
            df = df.reset_index(drop=True)
            df.to_csv(defile(data_path), index=False, header=None)
            showtext('插值完成', 'black')
    except Exception as e:
        showtext(str(e.__traceback__.tb_lineno) + str(e), 'red')
# 显示信息
def showtext(txt,color):
    text.tag_add('%s'%(color),'end')
    text.tag_config('%s'%(color), foreground=color)
    text.insert(INSERT, "%s\n"%(txt),'%s'%(color))
    text.see(END)
    log = open(log_path, 'a')
    log.write(str(datetime.datetime.now()) + "  " + txt + '\n')
    log.close()
# 清除信息
def cleartext():
    text.delete('1.0','end')
# mark点识别
def markdiscern():
    mark_path = os.path.dirname(askopenfilename(title='选择Mark点图片')) # 打开的图片路径
    if os.path.exists(mark_path+r'\Mark.png') and os.path.exists(mark_path+r'\mark.txt'):
        pd.set_option('mode.chained_assignment', None)
        mark = pd.read_csv(mark_path + r'\mark.txt', sep=' ', names=['code', 'u', 'v'], header=None)
        im = readimg(mark_path + r'\Mark.png', 1)  # 原始图片
        im2 = readimg(mark_path + r'\mask.png', 1)  # 遮罩
        img = cv2.bitwise_and(im, im2)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # 灰度转化
        saveimg(img,mark_path + r'\1Gray.png')
        ret_th, img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)  # 全局阈值分割(OTSU二值化)
        saveimg(img,mark_path + r'\2Binar.png')
        contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        cr = np.ones((len(contours), 3))  # 生成n行3列的数组
        for i in range(len(contours)):
            (x, y), (MA, ma), angle = cv2.fitEllipse(contours[i])
            c, r = round(x), round(y)  # 四舍五入
            for j in range(len(mark)):
                if abs(mark['u'].iloc[j] - x) + abs(mark['v'].iloc[j] - y) < 50:
                    mark['u'].iloc[j], mark['v'].iloc[j] = x, y
                    cv2.putText(im, str(mark['code'].iloc[j]), (c, r - 5), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2)
            cr[i] = x, y, i
            cv2.ellipse(im, ((x, y), (MA, ma), angle), (0, 0, 255), 1)  # 画椭圆（图像，（中心坐标，长短轴，旋转角度），颜色，线条粗细）
            cv2.line(im, (c - 5, r), (c + 5, r), (0, 255, 255), 1)  # 原图上画十字光标
            cv2.line(im, (c, r - 5), (c, r + 5), (0, 255, 255), 1)
        saveimg(im,mark_path + r'\2Discern.png')
        if os.path.exists(os.path.dirname(mark_path)+ r'\xyz.csv'):
            xyz = pd.read_csv(os.path.dirname(mark_path) + r'\xyz.csv', names=['x', 'y', 'z'], header=None)
            xyz.insert(loc=0, column='code', value=[i for i in range(len(xyz))])
            mark = pd.merge(mark, xyz, how='inner', on='code')
        mark.to_csv(defile(mark_path+r'\mark.csv'),index=False,header=None)
# 图片拖拽
def pic_canvas(files):
    global show_tkImage
    img = r'\n'.join((file.decode('gbk') for file in files))
    show_tkImage = PhotoImage(file=img)
    canvas.create_image(0, 0, anchor='nw', image=show_tkImage)
# 打开图片
def openimg(files):
    global g_image_original,g_image_show,g_image_zoom,show_tkImage,mousetype,img_path,pic_path,g_zoo,g_zoom,g_window_wh,\
        g_location_win,location_win,g_location_click,draw,pointss,kointss
    if mousetype == 0 or mousetype == 2 or mousetype == 5:
        if 'list' in str(type(files)):
            img_path = r'\n'.join((file.decode('gbk') for file in files))  # 打开的图片路径
        elif os.path.exists(str(files)):
            img_path = files
        else:
            img_path = askopenfilename(title='选择圆圈图片')  # 打开的图片路径
        g_zoom = 0.25
        pointss,kointss = [],[]
        g_location_win = [0, 0]  # 相对于大图，窗口在图片中的位置
        location_win = [0, 0]  # 鼠标左键点击时，暂存相对窗口位置
        g_location_click = [0, 0]  # 相对于窗口，鼠标左键点击和释放的位置
        pic_path = os.path.dirname(img_path)
        if os.path.exists(img_path):
            rd1.config(state='disabled')
            rd2.config(state='disabled')
            rd3.config(state='disabled')
            cob0.config(state='disabled')
            mousetype = 2
            g_image_original = readimg(img_path, 1) #打开的原始图片(3通道)
            (high,wide) = g_image_original.shape[:2]
            g_image_original = cv2.cvtColor(g_image_original, cv2.COLOR_BGR2RGB)
            draw = cv2.resize(np.zeros((wide, high, 3), np.uint8), (wide, high))
            g_zoo = 480/g_image_original.shape[0]
            g_zoom = 480/g_image_original.shape[0]
            g_window_wh = [round(g_zoo*g_image_original.shape[1]),480]
            canvas.place_forget()
            canvas.place(width=round(g_zoo*g_image_original.shape[1]), height=480, relx=0.15625, rely=0.25)
            g_image_show = cv2.resize(g_image_original, (0, 0), fx=g_zoo, fy=g_zoo) # 实际显示的图片
            g_image_zoom = g_image_show.copy()  # 缩放后的图片
            show_tkImage = ImageTk.PhotoImage(PIL.Image.fromarray(g_image_show))
            canvas.create_image(0, 0, anchor='nw', image=show_tkImage)
            # show_tkImage.paste(PIL.Image.fromarray(g_image_show))
# 线程打开图片
def run_openimg(files):
    check_ip = threading.Thread(target=openimg, args=(files,))
    check_ip.setDaemon(True)
    check_ip.start()
# 新建勾边
def newedge(event):
    global mousetype
    if mousetype == 2 or mousetype == 4:
        canvas.config(cursor="plus")
        mousetype = 3 #进入勾边模式
# 新建遮罩
def newshade(event):
    global mousetype
    if mousetype == 2 or mousetype == 4:
        canvas.config(cursor="tcross")
        mousetype = 3.5  # 进入遮罩模式
# 撤销上个点
def delpoint(event):
    global point,koint,markpoint
    event.widget.focus_set()
    if mousetype == 3:
        canvas.delete('firstline', 'lastline')
        del point[-2], point[-1]
        points = point[:]
        for i in range(int(len(point) / 2)):
            points[i * 2] = round(point[i * 2] * g_zoom) - g_location_win[0]
            points[i * 2 + 1] = round(point[i * 2 + 1] * g_zoom) - g_location_win[1]
        if len(points) > 2:
            canvas.create_line(points, fill='red', width=linew, tag='firstline')
            canvas.create_line(points[-2],points[-1],event.x, event.y, fill='red', width=linew, tag='lastline')
        elif len(points) == 2:
            canvas.create_line(points[0],points[1],event.x, event.y, fill='red', width=linew, tag='lastline')
    elif mousetype == 3.5:
        canvas.delete('firstkine', 'lastkine')
        del koint[-2], koint[-1]
        koints = koint[:]
        for i in range(int(len(koint) / 2)):
            koints[i * 2] = round(koint[i * 2] * g_zoom) - g_location_win[0]
            koints[i * 2 + 1] = round(koint[i * 2 + 1] * g_zoom) - g_location_win[1]
        if len(koints) > 2:
            canvas.create_line(koints, fill='green', width=linew, tag='firstkine')
            canvas.create_line(koints[-2],koints[-1],event.x, event.y, fill='green', width=2, tag='lastkine')
        elif len(koints) == 2:
            canvas.create_line(koints[0],koints[1],event.x, event.y, fill='green', width=2, tag='lastkine')
    elif mousetype == 1:
        if len(markpoint) > 0:
            del markpoint[-1]
            marknumval.set(marknumval.get() - 1)
# 取消勾边
def cancelmask(event):
    global mousetype
    canvas.config(cursor="arrow")
    if len(point) == 0:
        if len(pointss) == 0:
            mousetype = 2
        else:
            mousetype = 4
# 鼠标移动
def mousemove(event):
    global coo
    if current:
        coords = event.widget.coords(current) #记录上一次鼠标点下的坐标和随时移动的坐标
        if len(coords) == 4:
            coo = coords[:]
        coords = coo
        coords[2] = event.x
        coords[3] = event.y
        event.widget.coords(current, *coords)
    if mousetype == 3:
        canvas.delete('lastline')
        if len(point) >= 2:
            canvas.create_line(round(point[-2] * g_zoom) - g_location_win[0], round(point[-1] * g_zoom) - g_location_win[1],
                           event.x, event.y, fill='red', width=linew, tag='lastline')
    elif mousetype == 3.5:
        canvas.delete('lastkine')
        if len(koint) >= 2:
            canvas.create_line(round(koint[-2] * g_zoom) - g_location_win[0], round(koint[-1] * g_zoom) - g_location_win[1],
                           event.x, event.y, fill='green', width=2, tag='lastkine')
# 鼠标左键
def leftdown(event):
    global current,point,firstpoint,koint,firstkoint
    event.widget.focus_set()  # 焦点设置
    if mousetype == 3 or mousetype == 3.5:# 勾边/遮罩模式
        if (event.x+g_location_win[0])/g_zoom <= 4:
            x = 0
        elif (event.x+g_location_win[0])/g_zoom >= g_image_original.shape[1] -5:
            x = g_image_original.shape[1] - 1
        else:
            x = (event.x+g_location_win[0])/g_zoom
        if (event.y+g_location_win[1])/g_zoom <= 4:
            y = 0
        elif (event.y+g_location_win[1])/g_zoom >= g_image_original.shape[0] -5:
            y = g_image_original.shape[0] -1
        else:
            y = (event.y+g_location_win[1])/g_zoom
        if mousetype == 3:  # 勾边模式
            if current is None:
                firstpoint.append([event.x, event.y])
            point.append(x)
            point.append(y)
            canvas.delete('firstline','lastline')
            points = point[:]
            for i in range(int(len(point) / 2)):
                points[i * 2] = round(point[i * 2] * g_zoom) - g_location_win[0]
                points[i * 2 + 1] = round(point[i * 2 + 1] * g_zoom) - g_location_win[1]
            if len(points) > 2:
                canvas.create_line(points, fill='red', width=linew, tag='firstline')
        elif mousetype == 3.5:  # 遮罩模式
            if current is None:
                firstkoint.append([event.x, event.y])
            koint.append(x)
            koint.append(y)
            canvas.delete('firstkine','lastkine')
            koints = koint[:]
            for i in range(int(len(koint) / 2)):
                koints[i * 2] = round(koint[i * 2] * g_zoom) - g_location_win[0]
                koints[i * 2 + 1] = round(koint[i * 2 + 1] * g_zoom) - g_location_win[1]
            if len(koints) > 2:
                canvas.create_line(koints, fill='green', width=2, tag='firstkine')
# 双击左键
def mousedouble(event):
    global current,firstpoint,mousetype,firstkoint,point,koint,pointss,kointss
    event.widget.focus_set()  # 焦点设置
    if len(firstpoint) != 0 and len(point) > 4:
        canvas.create_line(round(point[-2] * g_zoom) - g_location_win[0],round(point[-1] * g_zoom) - g_location_win[1],
                           round(point[0] * g_zoom) - g_location_win[0],round(point[1] * g_zoom) - g_location_win[1],
                            fill='red', width=linew, tag='lastline')
        current = None
        canvas.config(cursor="arrow")
        firstpoint = []
        pointss.append(point)
        point = []
        mousetype = 4 #勾边完成模式
    elif len(firstkoint) != 0 and len(koint) > 4:
        canvas.create_line(round(koint[-2] * g_zoom) - g_location_win[0],round(koint[-1] * g_zoom) - g_location_win[1],
                           round(koint[0] * g_zoom) - g_location_win[0],round(koint[1] * g_zoom) - g_location_win[1],
                            fill='green', width=2, tag='lastkine')
        current = None
        canvas.config(cursor="arrow")
        firstkoint = []
        kointss.append(koint)
        koint = []
        mousetype = 4 #勾边完成模式
    kpoint(event)
# 鼠标中键
def middledown(event):
    global g_location_click, location_win
    event.widget.focus_set()
    if mousetype != 0 and mousetype != 1 and mousetype != 1.5 and mousetype != 5:
        g_location_click = [event.x, event.y]  # 中键点击时，暂存鼠标相对于窗口的坐标
        location_win = g_location_win[:]  # 鼠标左键点击时，暂存相对窗口位置
# 中键按下移动
def downmove(event):
    global g_image_show, g_image_zoom, g_location_win, g_zoom, show_tkImage
    event.widget.focus_set()
    if mousetype != 0 and mousetype != 1 and mousetype != 1.5 and mousetype != 5:
        h1, w1 = g_image_zoom.shape[0:2]  # 缩放图片的宽高
        w2, h2 = g_window_wh  # 窗口的宽高
        if w1 > w2 and h1 > h2:  # 图片的宽高大于窗口宽高，可左右上下移动
            show_wh = [w2, h2] # 实际显示图片的宽高
            g_location_win[0] = location_win[0] + g_location_click[0] - event.x
            g_location_win[1] = location_win[1] + g_location_click[1] - event.y
        else:  # 图片的宽高小于等于窗口宽高，无法移动
            show_wh = [w1, h1] # 实际显示图片的宽高
            g_location_win = [0, 0]
        g_location_win = check_location([w1, h1], [w2, h2], g_location_win)  # 矫正窗口在图片中的位置
        g_image_show = g_image_zoom[g_location_win[1]:g_location_win[1] + show_wh[1],
                       g_location_win[0]:g_location_win[0] + show_wh[0]]  # 实际显示的图片
        canvas.delete('all')
        show_tkImage = ImageTk.PhotoImage(PIL.Image.fromarray(g_image_show))
        canvas.create_image(0, 0, anchor='nw', image=show_tkImage)
        kpoint(event)
# 画出全部勾边和遮罩
def kpoint(event):
    points = point[:]
    for i in range(int(len(points) / 2)):
        points[i * 2] = round(point[i * 2] * g_zoom) - g_location_win[0]
        points[i * 2 + 1] = round(point[i * 2 + 1] * g_zoom) - g_location_win[1]
    if len(points) > 2:
        canvas.create_line(points, fill='red', width=linew, tag='firstline')
    if mousetype == 3:
        if len(point) >= 2:
            canvas.create_line(round(point[-2] * g_zoom) - g_location_win[0],round(point[-1] * g_zoom) - g_location_win[1],
                               event.x, event.y, fill='red', width=linew, tag='lastline')
    if len(pointss)>0:
        for j in pointss:
            points = j[:]
            for i in range(int(len(points) / 2)):
                points[i * 2] = round(j[i * 2] * g_zoom) - g_location_win[0]
                points[i * 2 + 1] = round(j[i * 2 + 1] * g_zoom) - g_location_win[1]
            canvas.create_line(points,points[0],points[1], fill='red', width=linew, tag='overline')
    koints = koint[:]
    for i in range(int(len(koint) / 2)):
        koints[i * 2] = round(koint[i * 2] * g_zoom) - g_location_win[0]
        koints[i * 2 + 1] = round(koint[i * 2 + 1] * g_zoom) - g_location_win[1]
    if len(koints) > 2:
        canvas.create_line(koints, fill='green', width=2, tag='firstkine')
    if mousetype == 3.5:
        if len(koint) >= 2:
            canvas.create_line(round(koint[-2] * g_zoom) - g_location_win[0],round(koint[-1] * g_zoom) - g_location_win[1],
                               event.x, event.y, fill='green', width=2, tag='lastkine')
    if len(kointss) > 0:
        for j in kointss:
            koints = j[:]
            for i in range(int(len(koints) / 2)):
                koints[i * 2] = round(j[i * 2] * g_zoom) - g_location_win[0]
                koints[i * 2 + 1] = round(j[i * 2 + 1] * g_zoom) - g_location_win[1]
            canvas.create_line(koints, koints[0], koints[1], fill='green', width=2, tag='overkine')
# 鼠标滚轮
def wheel(event):
    global g_image_show, g_image_zoom, g_location_win, g_zoom,show_tkImage,mousetype
    event.widget.focus_set()
    if mousetype != 0 and mousetype != 1 and mousetype != 1.5 and mousetype != 5:
        g_zoom_old = g_zoom  # 缩放前的缩放倍数，用于计算缩放后窗口在图片中的位置
        g_zoom = count_zoom(event.delta, g_step, g_zoom)  # 计算缩放倍数
        w1, h1 = [round(g_image_original.shape[1] * g_zoom), round(g_image_original.shape[0] * g_zoom)]  # 缩放图片的宽高
        w2, h2 = g_window_wh  # 窗口的宽高
        g_image_zoom = cv2.resize(g_image_original, (w1, h1), interpolation=cv2.INTER_AREA)  # 图片缩放
        if w1 >= w2 and h1 >= h2:  # 缩放后，图片宽高大于等于窗口宽高
            show_wh = [w2, h2] # 实际显示图片的宽高
        else:  # 缩放后，图片宽高小于窗口宽高
            show_wh = [w1, h1] # 实际显示图片的宽高
        g_location_win = [round((g_location_win[0] + event.x) * g_zoom / g_zoom_old - event.x),
                          round((g_location_win[1] + event.y) * g_zoom / g_zoom_old - event.y)]  # 缩放后，窗口在图片的位置
        g_location_win = check_location([w1, h1], [w2, h2], g_location_win)  # 矫正窗口在图片中的位置
        g_image_show = g_image_zoom[g_location_win[1]:g_location_win[1] + show_wh[1],
                       g_location_win[0]:g_location_win[0] + show_wh[0]]  # 实际的显示图片
        canvas.delete('all')
        show_tkImage = ImageTk.PhotoImage(PIL.Image.fromarray(g_image_show))
        canvas.create_image(0, 0, anchor='nw', image=show_tkImage)
        kpoint(event)
# 保存蒙版
def fillmask(event):
    overmask(0)
# 保存相机蒙版
def cammask(event):
    overmask(1)
# 保存蒙版并显示
def overmask(a):
    global point,mousetype,g_image_original,show_tkImage,g_zoom,g_location_win,location_win,g_location_click,koint,pointss,kointss,draw
    if mousetype == 4:
        canvas.delete("all")
        wide, high = g_image_original.shape[1], g_image_original.shape[0]
        if len(pointss)>0:
            for j in pointss:
                poly = np.zeros((int(len(j)/2),2),dtype=np.int_)
                for i in range(int(len(j)/2)):
                    poly[i] = round(j[2*i]), round(j[2*i+1])
                draw = cv2.fillPoly(draw, [poly], (255, 255, 255))
        else:
            draw[1:high - 2, 1:wide - 2, :] = 255
        if len(kointss) > 0:
            for j in kointss:
                koly = np.zeros((int(len(j) / 2), 2), dtype=np.int_)
                for i in range(int(len(j)/2)):
                    koly[i] = round(j[2*i]), round(j[2*i+1])
                draw = cv2.fillPoly(draw, [koly], (0, 0, 0))
        draw_show = cv2.resize(draw, (0, 0), fx=g_zoo, fy=g_zoo)
        show_tkImage = ImageTk.PhotoImage(PIL.Image.fromarray(draw_show))
        canvas.create_image(0, 0, anchor='nw', image=show_tkImage)
        mask = np.zeros(draw.shape, np.uint8)
        mask[1:high-2, 1:wide-2,:] = 255
        draw = cv2.bitwise_and(draw, mask)  # 贴合模板（按位与）
        if a == 0:
            saveimg(draw,os.path.dirname(img_path) + r'\mask.png')
            # txtfile = open(os.path.dirname(img_path) + r'\gb.txt', 'w')
        else:
            saveimg(draw,os.path.dirname(img_path) + r'\mask%s.png'%(os.path.splitext(img_path)[0][-1]))
        # print(point)
        # print(pointss)
        mousetype = 5
        point = []
        koint = []
        pointss = []
        kointss = []
        g_zoom = 0.25
        g_location_win = [0, 0]  # 相对于大图，窗口在图片中的位置
        location_win = [0, 0]  # 鼠标左键点击时，暂存相对窗口位置
        g_location_click = [0, 0]  # 相对于窗口，鼠标左键点击和释放的位置
# 打开勾边
def openmask(event):
    global g_image_original,mousetype,draw,pointss
    if mousetype == 2:
        maskfile = askopenfilename(title='选择勾边图片', filetype=[("勾边图片", "*mask.png")])
        if os.path.exists(maskfile):
            pointss.append([0,0])
            mask = readimg(maskfile, 0)  # 打开的原始图片
            draw = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            h, w = mask.shape[:2]
            m = np.zeros((h + 2, w + 2), np.uint8)
            contours1, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for i in contours1: #所有轮廓
                cv2.polylines(g_image_original, [i], True, (255, 0, 0), 2)
            cv2.floodFill(mask, m, (0, 0), 255)  # 漫水填充
            contours2, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            area = [[cv2.contourArea(i), i] for i in contours2]
            area.sort(key=lambda x: x[0], reverse=True)
            area = area[1:] #遮罩的轮廓
            for i in area:
                cv2.polylines(g_image_original, [i[1]], True, (0, 255, 0), 2)
# 标记mark点
def tabmark(event):
    global markpoint
    if marknumval.get() not in [i[0] for i in markpoint]:
        markpoint.append([marknumval.get(),event.x,event.y])
        marknumval.set(marknumval.get()+1)
# 自动勾边
def autoedge(a):
    global point, mousetype, g_image_original, show_tkImage, g_zoom, g_location_win, location_win, g_location_click, koint, pointss, kointss
    canvas.delete("all")
    white = readimg(os.path.dirname(img_path) + r'\white.png', 0)
    # white = cv2.medianBlur(cv2.cvtColor(g_image_original,cv2.COLOR_BGR2GRAY),3)
    ret_th, white = cv2.threshold(white, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    contours, hierarchy = cv2.findContours(white, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    area = [[cv2.contourArea(i), i] for i in contours]
    area.sort(key=lambda x: x[0], reverse=True)
    n = area[0][1]
    points = []
    points_app = points.append
    for i in n:
        points_app(i[0][0])
        points_app(i[0][1])
    pointss.append(points)
    mask,mask2 = np.zeros((white.shape[0], white.shape[1]), np.uint8),np.zeros((white.shape[0], white.shape[1]), np.uint8)
    mask = cv2.fillPoly(mask, [area[0][1]], 255)
    mask2[1:white.shape[0] - 2, 1:white.shape[1] - 2] = 255
    mask = cv2.bitwise_and(mask, mask2)  # 贴合模板（按位与）
    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    saveimg(mask,os.path.dirname(img_path) + r'\mask.png')
    cv2.polylines(g_image_original, [n], True, (255, 0, 0), 2)
    draw_show = cv2.resize(g_image_original, (0, 0), fx=g_zoo, fy=g_zoo)
    # draw_show = cv2.resize(mask, (0, 0), fx=g_zoo, fy=g_zoo)
    show_tkImage = ImageTk.PhotoImage(PIL.Image.fromarray(draw_show))
    canvas.create_image(0, 0, anchor='nw', image=show_tkImage)
    # mousetype = 5
    # point,koint,pointss,kointss = [],[],[],[]
    # g_zoom,g_location_win,location_win,g_location_click = 0.25,[0,0],[0,0],[0,0]
# 完成mark点勾边
def finishmark(event):
    global markpoint
    mark = [[],[],[]]
    for i in markpoint:
        mark[0].append(int(i[0]))
        mark[1].append(int(i[1]/0.25))
        mark[2].append(int(i[2]/0.25))
    marks = pd.DataFrame(mark, index=['code', 'x', 'y']).T
    marks.to_csv(mkdir('%s\%s\mark%s'%(paths,projectname,1))+r'\mark.txt',sep=' ',index=False,header=None)
    markpoint = []
# 鼠标右键
def rightdown(event):
    x,y =(event.x+g_location_win[0])/g_zoom,(event.y+g_location_win[1])/g_zoom
    menu = Menu(canvas, tearoff=False)  # 创建一个菜单
    if mousetype == 1:
        menu.add_command(label='标记Mark点',command=lambda: tabmark(event))
        if len(markpoint) > 0:
            menu.add_command(label='标记完成', command=lambda: finishmark(event))
    if mousetype != 1 and mousetype != 1.5 and mousetype != 3 and mousetype != 3.5 and mousetype != 4:
        menu.add_command(label='打开图片        Ctrl+Q', command=lambda: openimg(event))
    elif mousetype == 3 or mousetype == 3.5:
        if len(point) > 0 or len(koint) > 0:
            menu.add_command(label='撤销上个点     Ctrl+Z', command=lambda: delpoint(event))
        else:
            menu.add_command(label='取消',command=lambda:cancelmask(event))
    elif mousetype == 4:
        menu.add_command(label='保存蒙版        space', command=lambda: fillmask(event))
        menu.add_command(label='保存相机蒙版   Enter', command=lambda: cammask(event))
    if mousetype == 2 or mousetype == 4 or mousetype == 5:
        menu.add_command(label='关闭图片        Ctrl+X', command=lambda: closeimg(event))
    if mousetype == 2 or mousetype == 4:
        menu.add_command(label='新建勾边        Ctrl+A', command=lambda: newedge(event))
        menu.add_command(label='添加遮罩        Ctrl+S', command=lambda: newshade(event))
        menu.add_command(label='打开勾边        Ctrl+W', command=lambda: openmask(event))
        menu.add_command(label='亮度值:%s' % (g_image_original[round(y), round(x)]))
        menu.add_command(label='像素位置:(%s,%s)'%(round(x,5),round(y,5)),command=lambda: showxy(round(x,5),round(y,5)))
    if mousetype == 2:
        menu.add_command(label='自动勾边        Ctrl+I', command=lambda: autoedge(event))
    menu.post(event.x_root, event.y_root)
# 显示打开图片像素位置坐标
def showxy(x,y):
    showtext('%s\t%s'%(x,y),"green")
# 矫正窗口在图片中的位置 img_wh:图片的宽高, win_wh:窗口的宽高, win_xy:窗口在图片的位置
def check_location(img_wh, win_wh, win_xy):
    for i in range(2):
        if win_xy[i] < 0:
            win_xy[i] = 0
        elif win_xy[i] + win_wh[i] > img_wh[i]:
            win_xy[i] = img_wh[i] - win_wh[i]
    return win_xy
# 计算缩放倍数 flag：鼠标滚轮上移或下移的标识, step：缩放系数，滚轮每步缩放1.5, zoom：缩放倍数
def count_zoom(flag, step, zoom):
    if flag > 0:  # 滚轮上移
        zoom = zoom * step
        if zoom > 6:  # 最多只能放大到6倍
            zoom = 6
    else:  # 滚轮下移
        zoom = zoom / step
        if zoom < g_zoo:  # 最多只能缩小到0.25倍
            zoom = g_zoo
    zoom = round(zoom, 4)  # 取2位有效数字
    return zoom
# 关闭图片
def closeimg(event):
    global mousetype,g_location_win,location_win,g_location_click,g_zoom,point,koint,pointss,kointss
    canvas.delete('all')
    mousetype = 0
    point = []
    koint = []
    pointss = []
    kointss = []
    g_location_win = [0, 0]  # 相对于大图，窗口在图片中的位置
    location_win = [0, 0]  # 鼠标左键点击时，暂存相对窗口位置
    g_location_click = [0, 0]  # 相对于窗口，鼠标左键点击和释放的位置
    g_zoom= 0.25  # 图片缩放比例
    rd1.config(state='normal')
    rd2.config(state='normal')
    rd3.config(state='normal')
    cob0.config(state='normal')
    canvas.place_forget()
    if cammodel.get() == 'IDS':
        canvas.place(width=640, height=480, relx=0.15625, rely=0.25)
    else:
        canvas.place(width=720, height=480, relx=0.15625, rely=0.25)
# 莱卡xyz分列
def leica():
    leica_path = askopenfilename(title='选择莱卡xyz数据文件',filetype=[("", "*.csv")])  # 打开的csv路径
    if os.path.exists(leica_path) and leica_path[-3:] == 'csv':
        pd.set_option('mode.chained_assignment', None)
        Disto = pd.read_csv(leica_path,header=None)
        Disto = Disto[0].str.split(';', expand=True)
        first = firstval.get()
        xyz = Disto.loc[:, 2:5]
        xyz.dropna(subset=[2], inplace=True)
        xyz = xyz[xyz[2].str.contains('_')]
        xyz = xyz.drop([2], axis=1)
        xyz.columns = ['x', 'y', 'z']
        xyz = xyz.reset_index(drop=True)
        xyz = xyz.loc[first - 1:, :]
        xyz.to_csv(defile(os.path.dirname(leica_path)+r'\xyz.csv'), mode='a', index=False,header=None)
        os.startfile(os.path.dirname(leica_path))
# 线程圆心识别
def run_discern(distype,dome):
    pi_path = pic_path[:]
    try:
        if os.path.basename(pi_path) == projectname:
            boo = askyesno("提醒", "确定全部圆心识别？")
            if boo == True:
                path_list = [j for j in [os.path.join(pi_path, i) for i in os.listdir(pi_path)] if os.path.isdir(j) and isIP(os.path.basename(j).split('_')[0])]
                for i in path_list:
                    for j in os.listdir(i):
                        if distype == 0: #标准点
                            if 'cam' in j and 'Grid' not in j and 'New' not in j:
                                path = i + r'\%s'%(j)
                            else:
                                continue
                        elif distype == 1: #网格节点
                            if 'Gridcam' in j:
                                path = i + r'\%s' % (j)
                            else:
                                continue
                        elif distype == 2: #还原点
                            if 'Newcam' in j:
                                path = i + r'\%s' % (j)
                            else:
                                continue
                        else:
                            continue
                        t = threading.Thread(target=discern, args=(path,distype,dome,))
                        t.setDaemon(True)
                        t.start()
        else:
            cleartext()
            t = threading.Thread(target=discern,args=(pi_path,distype,dome,))
            t.setDaemon(True)
            t.start()
    except Exception as e:
        showtext(str(e.__traceback__.tb_lineno)+str(e), 'red')
# 判定函数
def judge_vertices(P, A, B, C):
    x_max = max(A[0], B[0], C[0])
    if P[0] > x_max+0.1:
        return False
    y_max = max(A[1], B[1], C[1])
    if P[1] > y_max+0.1:
        return False
    x_min = min(A[0], B[0], C[0])
    if P[0] < x_min-0.1:
        return False
    y_min = min(A[1], B[1], C[1])
    if P[1] < y_min-0.1:
        return False
    return True
# 判断是否在三角形内
def IsInside(A, B, C, P):
    # abc = np.vstack((A,B,C))#.reshape((3, 1, 2))
    # print(list(A))
    # print(np.array([int(A[0]),int(A[1])]))
    # print(abc,abc.shape,(P[0], P[1]))
    # a = (P[0],P[1])
    # print(cv2.pointPolygonTest(abc, a, False))
    # if cv2.pointPolygonTest(abc, a ,False) >= 0:
    #     return True
    # else:
    #     return False
    if not judge_vertices(P, A, B, C):
        return False
    a = np.round((B[0] - A[0]) * (P[1] - A[1]) - (B[1] - A[1]) * (P[0] - A[0]), 4)
    b = np.round((C[0] - B[0]) * (P[1] - B[1]) - (C[1] - B[1]) * (P[0] - B[0]), 4)
    if (a >= 0 and b < 0) or (a < 0 and b >= 0):
        return False
    else:
        c = np.round((A[0] - C[0]) * (P[1] - C[1]) - (A[1] - C[1]) * (P[0] - C[0]), 4)
        if (a >= 0 and c >= 0) or (a<= 0 and c <= 0):  # 判定是否在矩形内，若在则返回，不在继续循环
            return True
        else:
            return False
# 构造多边形
def create_triangle(column,row,uv):
    tri = []
    tri_app = tri.append
    for i in range(row - 1):
        for j in range(column - 1):
            a = [i * column + j, i * column + j + 1, (i + 1) * column + j + 1]
            if np.isnan(uv[0,a[0]]) or np.isnan(uv[0,a[1]]) or np.isnan(uv[0,a[2]]):
                pass
            else:
                tri_app(a)
            b = [i * column + j, (i + 1) * column + j + 1, (i + 1) * column + j]
            if np.isnan(uv[0,b[0]]) or np.isnan(uv[0,b[1]]) or np.isnan(uv[0,b[2]]):
                pass
            else:
                tri_app(b)
    return np.array(tri)
# 找三角形 mapping点cr,mapping点uv
def find_triangle(column,row,UV,uv):
    tri = []
    tri_app = tri.append
    for i in range(row-1):
        for j in range(column-1):
            a = [i * column + j, i * column + j + 1, (i + 1) * column + j + 1]
            if np.isnan(UV[0,a[0]]) or np.isnan(UV[0,a[1]]) or np.isnan(UV[0,a[2]]) or np.isnan(uv[0,a[0]]) or np.isnan(uv[0,a[1]]) or np.isnan(uv[0,a[2]]):
                pass
            else:
                tri_app(a)
            b = [i * column + j, (i + 1) * column + j + 1, (i + 1) * column + j]
            if np.isnan(UV[0,b[0]]) or np.isnan(UV[0,b[1]]) or np.isnan(UV[0,b[2]]) or np.isnan(uv[0,b[0]]) or np.isnan(uv[0,b[1]]) or np.isnan(uv[0,b[2]]):
                pass
            else:
                tri_app(b)
    return np.array(tri)
# src源图像网格坐标；dst目标图像网格坐标；fr_points源需要被映射的点。
def mapping_tri(src,dst,fr_points,tri,oridata):
    mapping_points = np.copy(oridata)
    fr_points_xy = np.vstack((fr_points,np.ones(fr_points.shape[1])))
    dst = np.float32(np.transpose(dst))
    src = np.float32(np.transpose(src))
    pts1 = [src[i,:] for i in tri] #pts1原图形三角顶点坐标
    H = [cv2.getAffineTransform(src[i,:], dst[i,:]) for i in tri] #仿射变换（原始图像的三个点，变换图像的三个点）
    for j in range(len(fr_points[0])):
        if ~np.isnan(fr_points[:, j][0]):
            for i,k in zip(pts1,H):
                if IsInside(i[0],i[1],i[2],fr_points[:, j]):
                    mapping_points[:, j] = np.dot(k, fr_points_xy[:, j])
                    break
    return mapping_points
# 平方和
def dist(p1,p2):
    dist = np.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)
    return dist
# 增加权重
def ad_point(p_sum,p_weight,dis,points_proj_map,points_came_map,proj_grid,cam_grid,weight,tri,dome):
    p_ad = np.copy(cam_grid[:,0:2])
    p_sum_weight = p_weight[:,0]*p_weight[:,1]
    p_sum_points = np.copy(p_sum)
    index_no_move =[] #增加未在拼接的点的索引
    pp = len(p_ad)
    pps = int(len(p_sum_points)/pp)
    for i in range(len(p_ad)):
        if ~np.isnan(p_ad[i,0]) and weight[i][0] !=0:
            temp_points ,temp_weight = [],[]
            if dome != 0:  # 非球幕
                for j in range(len(p_sum_points)):
                    if ~np.isnan(p_sum_points[j][0]) and p_sum_weight[j] != 0:
                        dist_p1 = dist(p_ad[i],p_sum_points[j])
                        if dist_p1 < dis:
                            temp_points.append(p_sum_points[j])
                            temp_weight.append(p_sum_weight[j])
            else:
                for j in range(pps):
                    if ~np.isnan(p_sum_points[j*pp+i][0]) and p_sum_weight[j*pp+i] != 0 and ~np.isnan(p_sum_weight[j*pp+i]):
                        dist_p1 = dist(p_ad[i],p_sum_points[j*pp+i])
                        if dist_p1 < dis:
                            temp_points.append(p_sum_points[j*pp+i])
                            temp_weight.append(p_sum_weight[j*pp+i])
            temp_points = np.array(temp_points)
            if len(temp_points) != 0:
                p_ad[i] = np.average(temp_points,weights=temp_weight,axis=0)
            if len(temp_points) <= 1:
                index_no_move.append(i)
        else:
            index_no_move.append(i)
    if dome != 0: #非球幕
        basic_node = mapping_tri(points_came_map, points_proj_map, cam_grid[:, 0:2].T, tri, proj_grid)
        proj_warp_t = mapping_tri(points_came_map, points_proj_map, p_ad.T, tri, basic_node)
    else: #球幕
        basic_node = proj_grid
        proj_warp_t = mapping_tri(cam_grid[:,0:2].T, proj_grid, p_ad.T, tri, basic_node)
    warp_delt = proj_warp_t - basic_node
    for j in index_no_move:
        warp_delt[:,j] = [0,0]
    for j in range(len(warp_delt[0])):
        if np.isnan(warp_delt[:, j][0]):
            warp_delt[:, j] = [0,0]
    return warp_delt

def gather_delt(delt,oridata,cam_width,cam_height,file_delt_out):
    try:
        result=np.copy(delt[0])
        for i in range(len(delt[0][0])):
            index=[]
            for j in range(len(delt)):
                if delt[j][:,i].all() != 0:
                    index.append(j)
            if len(index) == 0:
                pass
            elif len(index) == 1:
                result[:,i] = delt[index[0]][:,i]
            elif len(index) > 1:
                maxindex = index[0]
                maxd = dist_cam_center(oridata[index[0]][i][0],oridata[index[0]][i][1],cam_width,cam_height)
                for k in index:
                    d = dist_cam_center(oridata[k][i][0],oridata[k][i][1],cam_width,cam_height)
                    if d > maxd:
                        maxindex = k
                        maxd=d
                result[:,i]=delt[maxindex][:,i]
        np.savetxt(file_delt_out,(result.T),fmt='%.06f',delimiter=",")
    except Exception as e:
        showtext(str(e.__traceback__.tb_lineno)+str(e), 'red')

def dist_cam_center(u,v,width,height):
    if (u <= width) and (u >= 0) and (v >= 0) and (v <= height):
        u1 =(u / width * 1.0) * 2 if (u / width * 1.0) < (1 - u / width * 1.0) else (1 - u / width * 1.0) * 2
        v1 =(v / height * 1.0) * 2 if (v / height * 1.0) < (1 - v / height * 1.0) else (1 - v / height * 1.0) * 2
        if u1 == 0:
            u1 = u1 + 0.00000000001
        if v1 == 0:
            v1 = v1 + 0.00000000001
        d = (math.sin(math.pi * (u1 - 0.5)) + 1) * (math.sin(math.pi * (v1 - 0.5)) + 1) / 4.0
    else:
        d=0
    return d
# 读取txt
def read_txt_input(txt_path):
    with open(txt_path, 'r', encoding='utf-8') as file:
        # fp = open(txt_path)
        # ls=[line.strip('\n').split(' ') for line in fp]
        ls = file.read().split('\n')
        file.close()
    if ls[-1] == '':
        ls = ls[:-1]
    ls = pd.DataFrame(ls)
    ls = ls[0].str.split(' ', expand=True)
    ls = ls.replace('', 'Nan')
    ls = np.array(ls,dtype = float)   #将其转换成numpy的数组，并定义数据类型为float
    return ls
# 读取map数据
def map_input(f_cam_map):
    data = np.array(pd.read_csv(f_cam_map, header=None))
    points_proj_map = data[:, 2:4].T  # mapping点cr
    points_came_map = data[:, 4:6].T  # mapping点uv
    return points_proj_map, points_came_map
# 读取网格节点数据
def grid_input(f_cam_grid):
    cam_grid = np.array(pd.read_csv(f_cam_grid, header=None))[:, 4:6]  # 网格点uv
    return cam_grid
# 读取basic数据
def basic_input(f_proj_grid):
    proj_grid = read_txt_input(f_proj_grid).T[0:2]  # 网格点cr
    weight = read_txt_input(f_proj_grid)[:, 2:4]  # 网格点basic
    return proj_grid,weight
# 数据读取
def data_input(f_cam_map, f_cam_grid, f_proj_grid,dome):
    data = np.array(pd.read_csv(f_cam_map, header=None))
    points_proj_map = data[:, 2:4].T  # mapping点cr
    points_came_map = data[:, 4:6].T  # mapping点uv
    # data1 = np.array(pd.read_csv(f_cam_grid, header=None))
    cam_grid = np.array(pd.read_csv(f_cam_grid, header=None))[:, 4:6]  # 网格点uv
    proj_grid = read_txt_input(f_proj_grid).T[0:2]  # 网格点cr
    weight = read_txt_input(f_proj_grid)[:, 2:4]  # 网格点basic权重
    if dome == 0: #球幕
        ca = proj_grid[0].reshape((gridrow.get(), gridcolumn.get()))
        cb = proj_grid[1].reshape((gridrow.get(), gridcolumn.get()))
        proj_grids = interpolation(ca,cb,gridcolumn.get()-2,gridrow.get()-2,2,0,dome)
        proj_grid = np.vstack((proj_grids[0].reshape((1,gridrow.get()*gridcolumn.get())),proj_grids[1].reshape((1,gridrow.get()*gridcolumn.get()))))
    return points_proj_map, points_came_map, proj_grid, cam_grid, weight
# 数据读取
def data_input2(f_cam_map, f_cam_restore):
    data0 = np.array(pd.read_csv(f_cam_map, header=None))
    data1 = np.array(pd.read_csv(f_cam_restore, header=None))
    points_proj_map = data0[:, 2:4].T  # mapping点cr
    points_came_map = data0[:, 4:6].T  # mapping点uv
    points_came_restore = data1[:, 4:6].T  # 还原点uv
    return points_proj_map, points_came_map, points_came_restore

def warp_output(points_proj_map,points_came_map,points_came_t,f_texturKoord,pix_w,pix_h,tri1,tri2,file_out,dome):
    warp_o = read_txt_input(f_texturKoord).T[0:2]
    warp_o[0]=warp_o[0]*pix_w #归一化
    warp_o[1]=warp_o[1]*pix_h
    warp_oo = np.full(warp_o.shape, np.nan)
    camera_warp_o = mapping_tri(points_proj_map, points_came_map, warp_o, tri1,warp_oo)
    proj_warp_t = mapping_tri(points_came_t, points_proj_map, camera_warp_o, tri2,warp_oo)
    # print(proj_warp_t.shape)
    if dome != 0: #不是球幕
        pa = proj_warp_t[0].reshape((51, 81))
        pb = proj_warp_t[1].reshape((51, 81))
        paa, pbb = interpolation(pa, pb, 79, 49, 2,1,dome)
        proj_warp_t = np.vstack((paa.reshape((1, 51*81)),pbb.reshape((1, 51*81))))
    else: #是球幕
        pa = proj_warp_t[0].reshape((55, 85))
        pb = proj_warp_t[1].reshape((55, 85))
        paa, pbb = interpolation(pa, pb, 53, 83, 2,1,dome)
        proj_warp_t = np.vstack((paa.reshape((1, 55*85)),pbb.reshape((1, 55*85))))
    proj_warp_t[0] = proj_warp_t[0]/pix_w
    proj_warp_t[1] = proj_warp_t[1]/pix_h
    proj_warp_t= np.vstack((proj_warp_t,read_txt_input(f_texturKoord).T[2:7])).T
    np.savetxt(file_out,(proj_warp_t),fmt='%.06f')
    return proj_warp_t

def data_sum(warp,weight):
    point_sum=[]
    point_weight=[]
    for i in range(len(warp)):
        # warp[i] = warp[i][~np.isnan(warp[i]).any(axis=1)]  # 删除有空值的行
        # weight[i] = weight[i][~np.isnan(weight[i]).any(axis=1)]  # 删除有空值的行
        if(len(point_sum)==0):
            point_sum=np.copy(warp[i])
            point_weight=np.copy(weight[i])
        else:
            point_sum=np.vstack((np.copy(point_sum),np.copy(warp[i])))
            point_weight = np.vstack((np.copy(point_weight),np.copy(weight[i])))
    return point_sum,point_weight
# 自动修正
def autofix(dome,pi_path):
    try:
        path_list = [j for j in [os.path.join(pi_path,i) for i in os.listdir(pi_path)] if os.path.isdir(j) and isIP(os.path.basename(j).split('_')[0])]
        maplist = [os.path.join(i,'cam%s'%(k),'cam%s.csv'%(k)) for i in path_list for k in range(1, 4) if os.path.exists(os.path.join(i,'cam%s'%(k),'cam%s.csv'%(k))) ]
        gridlist = [os.path.join(i,'Gridcam%s'%(k),'cam%s.csv'%(k)) for i in path_list for k in range(1, 4) if os.path.exists(os.path.join(i,'Gridcam%s'%(k),'cam%s.csv'%(k))) ]
        basiclist = [os.path.join(i,'basic.txt') for i in path_list if os.path.exists(os.path.join(i,'basic.txt'))]
        if len(maplist) == 0:
            showtext("标准点不全！", 'red')
            return
        elif len(maplist) != len(gridlist):
            showtext("网格点和标准点文件数量不匹配！", 'red')
            return
        elif len(path_list) != len(basiclist):
            showtext("basic文件不全！", 'red')
            return
        showtext("开始自动修正", 'black')
        showtext("请等待......", 'black')
        route = len(path_list) #投影机数量
        CAMnum = int(len(maplist)/len(path_list))  # 相机数量
        data_cam,tri_cam = [],[]
        if dome != 0: #非球幕
            for i in range(route): #投影机数量
                data_num, trin = [], []
                # print(maplist,gridlist,basiclist)
                for j in range(CAMnum): #相机数量
                    data = data_input(maplist[i*CAMnum+j], gridlist[i*CAMnum+j],basiclist[i],dome)
                    # print(data[0].shape)
                    if data[0].shape[1] != (columnval.get()+2)*(rowval.get()+2):
                        showtext("投影机%s行列数不匹配"%(i),'red')
                        return
                    data_num.append(data)
                    trin.append(create_triangle(columnval.get() + 2,rowval.get() + 2, data[1]))
                data_cam.append(data_num)
                tri_cam.append(trin)
        else: #球幕
            for i in range(route): #投影机数量
                data_num, trin = [], []
                for j in range(CAMnum): #相机数量
                    # showtext(str(i)+' '+str(j),'red')
                    data = data_input(maplist[i*CAMnum+j], gridlist[i*CAMnum+j],basiclist[i],dome)
                    # if data[0].shape[1] != (columnval.get()+2)*(rowval.get()+2):
                    #     showtext("投影机%s行列数不匹配"%(i),'red')
                    #     return
                    data_num.append(data)
                    trin.append(find_triangle(gridcolumn.get(),gridrow.get(), data[3].T, data[2]))
                data_cam.append(data_num)
                tri_cam.append(trin)
        warp_all,weight_all,data_sum_cam = [],[],[]
        for j in range(CAMnum): #遍历相机
            warpn, weightn= [],[]
            for i in range(route): #遍历投影机
                warpn.append(data_cam[i][j][3])
                weightn.append(data_cam[i][j][4])
            data_sum_cam.append(data_sum(warpn, weightn))
            weight_all.append(weightn)
        camera_width,camera_height = camweival.get(),camheival.get() #相机分辨率宽，高
        for i in range(route): #遍历投影机
            dev_cam,oridata = [],[]
            for j in range(CAMnum):
                dev = ad_point(data_sum_cam[j][0], data_sum_cam[j][1], fixval.get(), data_cam[i][j][0],data_cam[i][j][1],
                               data_cam[i][j][2], data_cam[i][j][3],weight_all[j][i],tri_cam[i][j],dome)
                dev_cam.append(dev)
                oridata.append(data_cam[i][j][3])
            if os.path.exists(os.path.join(path_list[i] + r'\delt_' + str(i) + '.csv')):
                path = os.path.join(path_list[i] + r'\delt_' + str(i) + 's.csv')
            else:
                path = os.path.join(path_list[i] + r'\delt_' + str(i) + '.csv')
            gather_delt(dev_cam, oridata, camera_width, camera_height, path)
            showtext("已完成投影%s修正"%(i), 'black')
        showtext("自动修正完成！",'black')
    except Exception as e:
        showtext(str(e.__traceback__.tb_lineno)+str(e), 'red')
# 0是还原网格，1是计算网格
def restoregrid(dome,re):
    pi_path = pic_path[:]
    try:
        if os.path.basename(pi_path) == projectname: #整个项目计算/还原网格
        # if TRUE:
            path_list = [j for j in [os.path.join(pi_path, i) for i in os.listdir(pi_path)] if
                         os.path.isdir(j) and isIP(os.path.basename(j).split('_')[0])]
            if re == 0:#还原
                maplist = [os.path.join(i, 'cam%s' % (k), 'cam%s.csv' % (k)) for i in path_list for k in range(1, 4) if
                           os.path.exists(os.path.join(i, 'cam%s' % (k), 'cam%s.csv' % (k)))]
                restorelist = [os.path.join(i, 'Newcam%s' % (k), 'cam%s.csv' % (k)) for i in path_list for k in range(1, 4) if
                            os.path.exists(os.path.join(i, 'Newcam%s' % (k), 'cam%s.csv' % (k)))]
                bxlist = [os.path.join(i, 'data', '00texturKoord_0.txt') for i in path_list if os.path.exists(os.path.join(i, 'data', '00texturKoord_0.txt'))]
                # ratiolist = [os.path.join(i, 'cam%s' % (k), 'cam%s.txt' % (k)) for i in path_list for k in range(1, 4) if
                #            os.path.exists(os.path.join(i, 'cam%s' % (k), 'cam%s.txt' % (k)))]
            else:  # 计算左眼/右眼
                other_path = askdirectory(title='请选择需要计算项目的单眼原始文件夹')
                if not os.path.exists(other_path):
                    return
                if os.path.basename(other_path) == projectname:
                    showtext("请选择需要计算项目的单眼原始文件夹！", 'red')
                    return
                other_list = [j for j in [os.path.join(other_path, i) for i in os.listdir(other_path)] if
                             os.path.isdir(j) and isIP(os.path.basename(j).split('_')[0])]
                maplist = [os.path.join(i, 'cam%s' % (k), 'cam%s.csv' % (k)) for i in other_list for k in range(1, 4) if
                           os.path.exists(os.path.join(i, 'cam%s' % (k), 'cam%s.csv' % (k)))]
                restorelist = [os.path.join(i, 'cam%s' % (k), 'cam%s.csv' % (k)) for i in path_list for k in range(1, 4) if
                               os.path.exists(os.path.join(i, 'cam%s' % (k), 'cam%s.csv' % (k)))]
                bxlist = [os.path.join(i, 'data', '00texturKoord_0.txt') for i in other_list if
                          os.path.exists(os.path.join(i, 'data', '00texturKoord_0.txt'))]
                # ratiolist = [os.path.join(i, 'cam%s' % (k), 'cam%s.txt' % (k)) for i in other_list for k in range(1, 4)
                #              if os.path.exists(os.path.join(i, 'cam%s' % (k), 'cam%s.txt' % (k)))]
            if len(maplist) == 0:
                showtext("标准点不全！", 'red')
                return
            elif len(path_list) != len(bxlist):
                showtext("变形文件不全！", 'red')
                return
            elif len(maplist) != len(restorelist):
                showtext("标准点和还原点文件数量不匹配！", 'red')
                return
            if re == 0:  # 还原
                showtext("开始还原%s网格" % (projectname), 'black')
            else:
                showtext("开始计算%s网格" % (projectname), 'black')
            showtext("请等待......", 'black')
            route = len(path_list)  # 投影机数量
            CAMnum = int(len(maplist) / len(path_list))  # 相机数量
            for i in range(route): #遍历投影机(以识别点最多的为主屏，副屏取并集还原)
                warp = read_txt_input(bxlist[i])
                warp_o = warp.T[0:2]
                warp_o[0] = warp_o[0] * clientweival.get()  # 归一化
                warp_o[1] = warp_o[1] * clientheival.get()
                proj_warp_t = np.full(warp_o.shape, np.nan)
                warp_xy = np.vstack((warp_o, np.ones(warp_o.shape[1])))
                ptt1,ptt2 = [],[]
                for j in range(CAMnum): #遍历相机
                    mapcr, mapuv, storeuv = data_input2(maplist[i * CAMnum + j], restorelist[i * CAMnum + j])
                    if len(mapuv) == 0:
                        mapuv = np.full(mapcr.shape, np.nan)
                    if len(storeuv) == 0:
                        storeuv = np.full(mapcr.shape, np.nan)
                    df = pd.read_csv(maplist[i * CAMnum + j], header=None, names=['num', 'code', 'x', 'y', 'u', 'v'])
                    row, column = int(('00' + str(int(df['code'].iloc[-1])))[-4:-2]) - 1, int(
                        ('00' + str(int(df['code'].iloc[-1])))[-2:]) - 1
                    tri1 = create_triangle(column + 2, row + 2, mapuv)
                    tri2 = create_triangle(column + 2, row + 2, storeuv)
                    pts1 = [mapcr[:, i] for i in tri1]
                    pts2 = [storeuv[:, i] for i in tri2]
                    mapuv,mapcr,storeuv = mapuv.T,mapcr.T,storeuv.T
                    H1 = [cv2.getAffineTransform(np.float32(mapcr[i]), np.float32(mapuv[i])) for i in tri1]
                    H2 = [cv2.getAffineTransform(np.float32(storeuv[i]), np.float32(mapcr[i])) for i in tri2]
                    if j == 0:
                        ptt1.append([tri1, pts1, H1])
                        ptt2.append([tri2, pts2, H2])
                    else:
                        if len(tri1) > len(ptt1[0][0]):
                            ptt1.insert(0, [tri1, pts1, H1])
                        elif len(tri1) < len(ptt1[-1][0]):
                            ptt1.append([tri1, pts1, H1])
                        else:
                            for p in range(len(ptt1) - 1):
                                if len(tri1) < len(ptt1[p][0]) and len(tri1) > len(ptt1[p + 1][0]):
                                    ptt1.insert(p + 1, [tri1, pts1, H1])
                        if len(tri2) > len(ptt2[0][0]):
                            ptt2.insert(0, [tri2, pts2, H2])
                        elif len(tri2) < len(ptt2[-1][0]):
                            ptt2.append([tri2, pts2, H2])
                        else:
                            for p in range(len(ptt2) - 1):
                                if len(tri2) < len(ptt2[p][0]) and len(tri2) > len(ptt2[p + 1][0]):
                                    ptt2.insert(p + 1, [tri2, pts2, H2])
                (pa1, pb1, pc1), (pa2, pb2, pc2) = ptt1[0], ptt2[0]
                for j in range(len(ptt1)):  # 遍历相机
                    if j != 0:
                        for p, t, k in zip(ptt1[j][0], ptt1[j][1], ptt1[j][2]):
                            if p not in pa1:
                                pa1 = np.vstack((pa1, p))
                                pb1.append(t)
                                pc1.append(k)
                        for p, t, k in zip(ptt2[j][0], ptt2[j][1], ptt2[j][2]):
                            if p not in pa2:
                                pa2 = np.vstack((pa2, p))
                                pb2.append(t)
                                pc2.append(k)
                for k in range(warp_o.shape[1]):  # 遍历变形点每一列
                    for j, p in zip(pb1, pc1):
                        if IsInside(j[:, 0], j[:, 1], j[:, 2], warp_o[:, k]):
                            proj_warp_t[:, k] = np.dot(p, warp_xy[:, k])
                proj_warp_xy = np.vstack((proj_warp_t, np.ones(proj_warp_t.shape[1])))
                warp_oo = np.full(warp_o.shape, np.nan)
                for k in range(proj_warp_t.shape[1]):
                    if ~np.isnan(proj_warp_t[:, k][0]):
                        for j, p in zip(pb2, pc2):
                            if IsInside(j[:, 0], j[:, 1], j[:, 2], proj_warp_t[:, k]):
                                warp_oo[:, k] = np.dot(p, proj_warp_xy[:, k])
                warp_ot = pd.DataFrame(np.vstack((warp_oo, warp.T[2:7])).T,columns=['u','v','c','r','x','y','z'])
                warp_ot['code'] = range(len(warp_ot))
                # if dome != 0:
                #     r, c = 49, 79
                # else:
                #     r, c = 53, 83
                # calx = np.array(warp_ot['u']).reshape((r + 2, c + 2))
                # caly = np.array(warp_ot['v']).reshape((r + 2, c + 2))
                # calx, caly = interpolation(calx, caly, c, r, 2, 0, dome)  # 插两圈
                # warp_ot['u'] = [i for j in calx for i in j]
                # warp_ot['v'] = [i for j in caly for i in j]
                warp_ot1 = warp_ot.dropna(axis=0, subset=['u']) #非空点
                warp_ot2 = warp_ot[warp_ot[['u']].isnull().T.any()] #空点
                warp_ot2 = interp2D(warp_ot1, warp_ot2, 'c', 'r', 'u', 'v','quintic') #二维插值
                warp_ot = warp_ot1.append(warp_ot2).sort_values(by='code')
                warp_ot.loc[:,'u'], warp_ot.loc[:,'v'] = warp_ot.loc[:,'u']/clientweival.get(), warp_ot.loc[:,'v']/clientheival.get()
                warp_ot.drop('code', axis=1, inplace=True)
                bxout = np.array(warp_ot)
                if re == 0:  # 还原
                    np.savetxt(os.path.join(path_list[i], '00texturKoord_0.txt'),bxout,fmt='%.06f')
                    np.savetxt(os.path.join(mkdir(os.path.join(pi_path,'NewOUT')),'00texturKoord_%s.txt' % (i)),bxout,fmt='%.06f')
                    showtext("还原%s网格完成！"%(os.path.basename(path_list[i])), 'black')
                else:  # 计算左眼/右眼
                    copyfi(os.path.join(other_list[i],'0.png'),os.path.join(path_list[i],'0.png'))
                    copyfi(os.path.join(other_list[i],'20180423.xml'),os.path.join(path_list[i],'20180423.xml'))
                    np.savetxt(os.path.join(mkdir(os.path.join(path_list[i],'data')),'00texturKoord_0.txt'),bxout,fmt='%.06f')
                    np.savetxt(os.path.join(mkdir(os.path.join(pi_path,'OUT')),'00texturKoord_%s.txt'%(i)),bxout,fmt='%.06f')
                    showtext("计算%s网格完成！"%(os.path.basename(path_list[i])), 'black')
            if re == 0:  # 还原
                showtext("还原全部网格完成！", 'black')
            else:
                showtext("计算全部网格完成！", 'black')
        else: #单个屏幕计算/还原网格
            a = 0
            if re == 0:  # 还原
                other_list = [j for j in [os.path.join(os.path.dirname(pi_path), i) for i in os.listdir(os.path.dirname(pi_path))] if
                              os.path.isdir(j) and isIP(os.path.basename(j).split('_')[0])]
                for i in range(len(other_list)):
                    if os.path.basename(other_list[i]) == os.path.basename(pi_path):
                        a = i
                        break
                maplist = [os.path.join(pi_path,'cam%s'%(k),'cam%s.csv'%(k)) for k in range(1,4) if
                           os.path.exists(os.path.join(pi_path,'cam%s'%(k),'cam%s.csv'%(k)))]
                restorelist = [os.path.join(pi_path,'Newcam%s'%(k),'cam%s.csv'%(k)) for k in range(1,4) if
                               os.path.exists(os.path.join(pi_path,'Newcam%s'%(k),'cam%s.csv'%(k)))]
                bxlist = os.path.join(pi_path, 'data', '00texturKoord_0.txt')
                # ratiolist = [os.path.join(pi_path, 'cam%s' % (k), 'cam%s.txt' % (k)) for k in range(1,4) if
                #              os.path.exists(os.path.join(pi_path, 'cam%s' % (k), 'cam%s.txt' % (k)))]
            else:  # 计算左眼/右眼
                other_path = askdirectory(title='请选择需要计算单屏的单眼原始文件夹')
                if not os.path.exists(other_path):
                    return
                if os.path.basename(other_path) == projectname:
                    showtext("请选择需要计算单屏的单眼原始文件夹！", 'red')
                    return
                other_list = [j for j in [os.path.join(os.path.dirname(other_path), i) for i in os.listdir(os.path.dirname(other_path))] if
                              os.path.isdir(j) and isIP(os.path.basename(j).split('_')[0])]
                for i in range(len(other_list)):
                    if os.path.basename(other_list[i]) == os.path.basename(other_path):
                        a = i
                        break
                maplist = [os.path.join(other_path,'cam%s'%(k),'cam%s.csv'%(k))for k in range(1,4) if
                           os.path.exists(os.path.join(other_path,'cam%s'%(k),'cam%s.csv'%(k)))]
                restorelist = [os.path.join(pi_path,'cam%s'%(k),'cam%s.csv'%(k)) for k in range(1,4) if
                               os.path.exists(os.path.join(pi_path,'cam%s'%(k),'cam%s.csv'%(k)))]
                bxlist = os.path.join(other_path, 'data', '00texturKoord_0.txt')
                # ratiolist = [os.path.join(other_path,'cam%s'%(k),'cam%s.txt'%(k)) for k in range(1,4) if
                #              os.path.exists(os.path.join(other_path,'cam%s'%(k),'cam%s.txt'%(k)))]
            if len(maplist) == 0:
                showtext("标准点文件不全！", 'red')
                return
            elif not os.path.exists(bxlist):
                showtext("变形文件不全！", 'red')
                return
            elif len(maplist) != len(restorelist):
                showtext("标准点和还原点文件数量不匹配！", 'red')
                return
            if re == 0:  # 还原
                showtext("开始还原%s网格" % (os.path.basename(pi_path)), 'black')
            else:
                showtext("开始计算%s网格" % (os.path.basename(pi_path)), 'black')
            showtext("请等待......", 'black')
            CAMnum = len(maplist) # 相机数量
            warp = read_txt_input(bxlist)
            warp_o = warp.T[0:2]
            warp_o[0] = warp_o[0] * clientweival.get()  # 归一化
            warp_o[1] = warp_o[1] * clientheival.get()
            proj_warp_t = np.full(warp_o.shape, np.nan)
            warp_xy = np.vstack((warp_o, np.ones(warp_o.shape[1])))
            ptt1, ptt2 = [], []
            for j in range(CAMnum):  # 遍历相机
                mapcr, mapuv, storeuv = data_input2(maplist[j], restorelist[j])
                if len(mapuv) == 0:
                    mapuv = np.full(mapcr.shape, np.nan)
                if len(storeuv) == 0:
                    storeuv = np.full(mapcr.shape, np.nan)
                df = pd.read_csv(maplist[j], header=None,names=['num', 'code', 'x', 'y', 'u', 'v'])
                row, column = int(('00' + str(int(df['code'].iloc[-1])))[-4:-2]) - 1, int(
                    ('00' + str(int(df['code'].iloc[-1])))[-2:]) - 1
                tri1 = create_triangle(column + 2, row + 2, mapuv)
                tri2 = create_triangle(column + 2, row + 2, storeuv)
                pts1 = [mapcr[:, i] for i in tri1]
                pts2 = [storeuv[:, i] for i in tri2]
                mapuv, mapcr, storeuv = mapuv.T, mapcr.T, storeuv.T
                H1 = [cv2.getAffineTransform(np.float32(mapcr[i]), np.float32(mapuv[i])) for i in tri1]
                H2 = [cv2.getAffineTransform(np.float32(storeuv[i]), np.float32(mapcr[i])) for i in tri2]
                if j == 0:
                    ptt1.append([tri1, pts1, H1])
                    ptt2.append([tri2, pts2, H2])
                else:
                    if len(tri1) > len(ptt1[0][0]):
                        ptt1.insert(0,[tri1, pts1, H1])
                    elif len(tri1) < len(ptt1[-1][0]):
                        ptt1.append([tri1, pts1, H1])
                    else:
                        for p in range(len(ptt1)-1):
                            if len(tri1) < len(ptt1[p][0]) and len(tri1) > len(ptt1[p+1][0]):
                                ptt1.insert(p+1,[tri1, pts1, H1])
                    if len(tri2) > len(ptt2[0][0]):
                        ptt2.insert(0, [tri2, pts2, H2])
                    elif len(tri2) < len(ptt2[-1][0]):
                        ptt2.append([tri2, pts2, H2])
                    else:
                        for p in range(len(ptt2) - 1):
                            if len(tri2) < len(ptt2[p][0]) and len(tri2) > len(ptt2[p+1][0]):
                                ptt2.insert(p+1, [tri2, pts2, H2])
            (pa1, pb1, pc1),(pa2, pb2, pc2) = ptt1[0],ptt2[0]
            for j in range(len(ptt1)):  # 遍历相机
                if j != 0:
                    for p,t,k in zip(ptt1[j][0],ptt1[j][1],ptt1[j][2]):
                        if p not in pa1:
                            pa1 = np.vstack((pa1, p))
                            pb1.append(t)
                            pc1.append(k)
                    for p,t,k in zip(ptt2[j][0],ptt2[j][1],ptt2[j][2]):
                        if p not in pa2:
                            pa2 = np.vstack((pa2, p))
                            pb2.append(t)
                            pc2.append(k)
            for k in range(warp_o.shape[1]):  # 遍历变形点每一列
                for j,p in zip(pb1,pc1):
                    if IsInside(j[:, 0], j[:, 1], j[:, 2], warp_o[:, k]):
                        proj_warp_t[:, k] = np.dot(p, warp_xy[:, k])
            proj_warp_xy = np.vstack((proj_warp_t, np.ones(proj_warp_t.shape[1])))
            warp_oo = np.full(warp_o.shape, np.nan)
            for k in range(proj_warp_t.shape[1]):
                if ~np.isnan(proj_warp_t[:, k][0]):
                    for j,p in zip(pb2,pc2):
                        if IsInside(j[:, 0], j[:, 1], j[:, 2], proj_warp_t[:, k]):
                            warp_oo[:, k] = np.dot(p, proj_warp_xy[:, k])
            warp_ot = pd.DataFrame(np.vstack((warp_oo, warp.T[2:7])).T, columns=['u', 'v', 'c', 'r', 'x', 'y', 'z'])
            warp_ot['code'] = range(len(warp_ot))
            # if dome != 0:
            #     r, c = 49, 79
            # else:
            #     r, c = 53, 83
            # calx = np.array(warp_ot['u']).reshape((r + 2, c + 2))
            # caly = np.array(warp_ot['v']).reshape((r + 2, c + 2))
            # calx, caly = interpolation(calx, caly, c, r, 1, 0, dome)  # 插两圈
            # warp_ot['u'] = [i for j in calx for i in j]
            # warp_ot['v'] = [i for j in caly for i in j]
            warp_ot1 = warp_ot.dropna(axis=0, subset=['u']) #非空点
            warp_ot2 = warp_ot[warp_ot[['u']].isnull().T.any()] #空点
            warp_ot2 = interp2D(warp_ot1, warp_ot2, 'c', 'r', 'u', 'v','quintic') #二维插值
            warp_ot = warp_ot1.append(warp_ot2).sort_values(by='code')
            warp_ot.loc[:, 'u'], warp_ot.loc[:, 'v'] = warp_ot.loc[:, 'u'] / clientweival.get(), warp_ot.loc[:,'v'] / clientheival.get()
            warp_ot.drop('code', axis=1, inplace=True)
            bxout = np.array(warp_ot)
            if re == 0:  # 还原
                np.savetxt(os.path.join(pi_path, '00texturKoord_0.txt'),bxout, fmt='%.06f')
                np.savetxt(os.path.join(mkdir(os.path.join(os.path.dirname(pi_path), 'NewOUT')), '00texturKoord_%s.txt'%(a)),bxout, fmt='%.06f')
                showtext("还原网格完成！", 'black')
            else:  # 计算左眼/右眼
                copyfi(os.path.join(other_path, '0.png'), os.path.join(pi_path, '0.png'))
                copyfi(os.path.join(other_path, '20180423.xml'), os.path.join(pi_path, '20180423.xml'))
                np.savetxt(os.path.join(mkdir(os.path.join(pi_path, 'data')), '00texturKoord_0.txt'), bxout,fmt='%.06f')
                np.savetxt(os.path.join(mkdir(os.path.join(os.path.dirname(pi_path), 'OUT')), '00texturKoord_%s.txt' % (a)), bxout,fmt='%.06f')
                showtext("计算网格完成！", 'black')
    except Exception as e:
        showtext(str(e.__traceback__.tb_lineno)+str(e), 'red')
# 标定还原
def restorecalibration():
    try:
        # start = time.time()
        pi_path = pic_path
        if len(tree.focus()) != 0 and 'sceen' not in tree.focus():
            showtext("请选择整个场景！", 'red')
            return
        else:
            bd_path = askopenfilename(title='选择需要还原的标定文件',filetype=[("","*.csv")])
            if os.path.exists(bd_path):
                path_list = [j for j in [os.path.join(pi_path, i) for i in os.listdir(pi_path)] if
                             os.path.isdir(j) and isIP(os.path.basename(j).split('_')[0])]
                maplist = [os.path.join(i, 'cam%s' % (k), 'cam%s.csv' % (k)) for i in path_list for k in range(1, 4) if
                           os.path.exists(os.path.join(i, 'cam%s' % (k), 'cam%s.csv' % (k)))]
                restorelist = [os.path.join(i, 'Newcam%s' % (k), 'cam%s.csv' % (k)) for i in path_list for k in
                               range(1, 4) if os.path.exists(os.path.join(i, 'Newcam%s' % (k), 'cam%s.csv' % (k)))]
                bd = np.array(pd.read_csv(bd_path, header=None)).T
                bdnum = int(bd.shape[1]/len(path_list)) #每屏点数
                if len(maplist) == 0:
                    showtext("标准点不全！", 'red')
                    return
                elif len(maplist) != len(restorelist):
                    showtext("标准点和还原点文件数量不匹配！", 'red')
                    return
                elif bd.shape[0] != 2:
                    showtext("标定列数不为2！", 'red')
                    return
                route = len(path_list)  # 投影机数量
                CAMnum = int(len(maplist) / len(path_list))  # 相机数量
                for i in range(route): #遍历投影机
                    trilists = []
                    ptss,Hs = [],[]
                    warp_o = bd[:, bdnum * i:bdnum * (i + 1)]
                    proj_warp_t = np.full(warp_o.shape, np.nan)
                    warp_xy = np.vstack((warp_o, np.ones(warp_o.shape[1])))
                    for j in range(CAMnum): #遍历相机
                        (mapcr, mapuv, storeuv) = data_input2(maplist[i*CAMnum+j],restorelist[i*CAMnum+j])
                        if len(mapuv) ==0:
                            mapuv = np.full(mapcr.shape, np.nan)
                        if len(storeuv) ==0:
                            storeuv = np.full(mapcr.shape, np.nan)
                        if (columnval.get() + 2) * (rowval.get() + 2) != mapuv.shape[1]:
                            showtext("行列数不匹配！", 'red')
                            return
                        tri1 = create_triangle(columnval.get() + 2, rowval.get() + 2, mapuv)
                        tri2 = create_triangle(columnval.get() + 2, rowval.get() + 2, storeuv)
                        pts1 = [mapcr[:, i] for i in tri1]
                        ptss.append([storeuv[:, i] for i in tri2])
                        trilist = []
                        H = [cv2.getAffineTransform(np.float32(mapcr[:,i]).T, np.float32(mapuv[:,i]).T) for i in tri1]
                        Hs.append([cv2.getAffineTransform(np.float32(storeuv[:,i]).T, np.float32(mapcr[:,i]).T) for i in tri2])
                        for k in range(warp_o.shape[1]): #遍历标定点每一列
                            for p,t in zip(pts1,H): #遍历每个三角形
                                if IsInside(p[:,0], p[:,1], p[:,2], warp_o[:, k]):
                                    cambd = np.dot(t, warp_xy[:, k])
                                    trilist.append([j,k,cambd,segment(cambd,(camweival.get()/2,camheival.get()/2))])
                                    break
                            if len(trilist) != k+1:
                                trilist.append([j, k, 0, 10000])
                        trilists.append(trilist)
                    bd_list = []
                    for k in range(warp_o.shape[1]): #遍历标定点每一列
                        # minxy = [0,0,0,0]
                        for j in range(len(trilists)):
                            if j == 0:
                                minxy = trilists[j][k]
                            elif trilists[j][k][3] < minxy[3]:
                                minxy = trilists[j][k]
                        if minxy[3] != 0 and minxy[3] != 10000:
                            bd_list.append(minxy)
                    if len(bd_list) != bdnum:
                        showtext("标定未被相机完全拍到！", 'red')
                        return
                    for j in bd_list:
                        warp_xy = np.hstack((j[2], np.ones(1)))
                        for p,t in zip(ptss[j[0]],Hs[j[0]]):
                            if IsInside(p[:,0],p[:,1],p[:,2],j[2]):
                                proj_warp_t[:,j[1]] = np.dot(t,warp_xy)
                    if i == 0:
                        proj_warp = proj_warp_t.T
                    else:
                        proj_warp = np.vstack((proj_warp, proj_warp_t.T))
                file_out = os.path.join(os.path.dirname(bd_path),os.path.basename(bd_path)[:-4]+'New.csv')
                np.savetxt(file_out, (proj_warp), fmt='%.06f',delimiter=",")
                showtext("标定还原完成！", 'black')
        # end = time.time()
        # print(end-start)
    except Exception as e:
        showtext(str(e.__traceback__.tb_lineno)+str(e), 'red')
# 线程自动修正
def run_autofix(dome):
    try:
        pi_path = pic_path[:]
        if os.path.basename(pi_path) == projectname:
        # if True:
            boo = askyesno("提醒", "确定开始自动修正？")
            if boo == True:
                t = threading.Thread(target=autofix,args=(dome,pi_path,))
                t.setDaemon(True)
                t.start()
    except Exception as e:
        showtext(str(e.__traceback__.tb_lineno)+str(e), 'red')
# 线程计算网格 0是还原网格，1是计算网格
def run_restoregrid(dome,re):
    try:
        t = threading.Thread(target=restoregrid,args=(dome,re,))
        t.setDaemon(True)
        t.start()
    except Exception as e:
        showtext(str(e.__traceback__.tb_lineno)+str(e), 'red')
# 线程生成融合模板
def run_domefuse():
    try:
        t = threading.Thread(target=domefuse)
        t.setDaemon(True)
        t.start()
    except Exception as e:
        showtext(str(e.__traceback__.tb_lineno)+str(e), 'red')
# k:圆削几圈
def rounding(pic,add_im,k):
    # if k == 0:
    #     return pic,add_im
    ret_th, add = cv2.threshold(add_im, 127, 255, cv2.THRESH_BINARY)
    for a in range(k):
        contours, hierarchy = cv2.findContours(add, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        area = [[cv2.contourArea(j),j] for j in contours]
        area.sort(key=lambda x:x[0],reverse=True)
        add[area[0][1][:,:,1], area[0][1][:,:,0]] = 0
        for i in range(len(pic)):
            pic[i][area[-1][1][:,:,1], area[-1][1][:,:,0]] = 0
    add = np.zeros((pic[0].shape[0], pic[0].shape[1]), np.uint8)
    pics = []
    for i in range(len(pic)):
        add = cv2.add(add,pic[i])
        ret_th, pic[i] = cv2.threshold(pic[i], 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)  # 全局阈值分割(OTSU二值化)
        contours, hierarchy = cv2.findContours(pic[i], cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        area = [[cv2.contourArea(j),j] for j in contours]
        area.sort(key=lambda x:x[0],reverse=True)
        pics.append([pic[i],area[0][1]])
    return pics,add
# 线程运行函数，获得返回值
class MyThread(threading.Thread):
    def __init__(self, func, args=()):
        super(MyThread, self).__init__()
        self.func = func
        self.args = args

    def run(self):
        self.result = self.func(*self.args)

    def get_result(self):
        try:
            return self.result
        except Exception:
            return None
# 生成融合模板
def domefuse():
    path = pic_path[:]
    try:
        # if True:
        # if os.path.basename(path) == projectname:
        mkdir(path+r'\map')
        path_list=[j for j in [os.path.join(path,i) for i in os.listdir(path)] if os.path.isdir(j) and isIP(os.path.basename(j).split('_')[0])]
        imagenum = len(path_list)

        if os.path.exists(path+r'\map\0.png'):
            im = readimg(path+r'\map\0.png', 1)  # 原始图片
            ad = 1
        elif os.path.exists(path_list[0] + r'\cam1\white.png'):
            im = readimg(path_list[0] + r'\cam1\white.png', 1)  # 原始图片
            ad = 0
        else:
            showtext("图片不存在！", 'red')
            return
        showtext("开始生成融合模板", 'black')
        showtext("请等待......", 'black')
        im = cv2.resize(im, (0, 0), fx=fuseval.get(), fy=fuseval.get(), interpolation=cv2.INTER_AREA)
        camwei, camhei = im.shape[1],im.shape[0]
        pic = []
        add_im = np.zeros((camhei,camwei), np.uint8)
        for i in range(imagenum):
            if ad == 0:
                im = readimg(path_list[i]+r'\cam1\white.png',0) #原始图片
                mask = readimg(path_list[i]+r'\cam1\mask.png',0) #蒙版
                img = cv2.bitwise_and(im,mask) #贴合模板（按位与）
                img = cv2.medianBlur(img, 3)
                img = cv2.convertScaleAbs(img,alpha=1.5,beta=0)
            else:
                img = readimg(path + r'\map\%s.png'%(i), 0)  # 原始图片
            img = cv2.resize(img, (0, 0), fx=fuseval.get(), fy=fuseval.get(), interpolation=cv2.INTER_AREA)
            ret_th, img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)  # 全局阈值分割(OTSU二值化)
            contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            area = [[cv2.contourArea(i), i] for i in contours]
            area.sort(key=lambda x: x[0], reverse=True)
            black = np.zeros((camhei, camwei), np.uint8)
            img = cv2.fillPoly(black, [area[0][1]], 128)
            add_im = cv2.add(add_im,img)
            img[img == 128] = 255
            pic.append((img,area[0][1]))
        saveimg(add_im,path+r'\map\add.png')
        cor_list = []
        cors = [[0,0],[camwei,0],[camwei,camhei],[0,camhei]]
        cort = list(itertools.permutations([0,1,2,3],4))
        for i in range(len(pic)):
            cor = cv2.goodFeaturesToTrack(pic[i][0],4,0.01,10,blockSize=7)
            # cor = cv2.goodFeaturesToTrack(pic[i][0],4,0.01,10,blockSize=5,useHarrisDetector=True)
            cor = np.int0(cor)
            # img = cv2.cvtColor(pic[i][0], cv2.COLOR_GRAY2BGR)
            # img[cor[:, :, 1], cor[:, :, 0]] = [0, 0, 255]
            # saveimg(img, path + r'\map\%s.png' % (i))
            e = []
            for j in range(len(cort)):
                a = segment(cors[0], cor[cort[j][0], 0, :])
                b = segment(cors[1], cor[cort[j][1], 0, :])
                c = segment(cors[2], cor[cort[j][2], 0, :])
                d = segment(cors[3], cor[cort[j][3], 0, :])
                e.append([a+b+c+d,j])
            e.sort(key=lambda x:x[0])
            h = []
            for j in cort[e[0][1]]:
                f = []
                f_app = f.append
                for k in range(len(pic[i][1])):
                    g = segment(cor[j][0], pic[i][1][k][0])
                    f_app([g,k])
                f.sort(key=lambda x:x[0])
                h.append(f[0][1])
            cor_list.append([[list(pic[i][1][k][0]) for k in h],h])
        outer = []
        for i in range(len(pic)):
            img = cv2.cvtColor(pic[i][0], cv2.COLOR_GRAY2BGR)
            length,ptsu = len(pic[i][1]),[]
            for j in range(4):
                pts = []
                pts.append(cors[j-1])
                pts.append(cors[j])
                a,b = cor_list[i][1][j],cor_list[i][1][j-1]
                if a<b:
                    if b==length-1:
                        pts.extend(pic[i][1][a:,0].tolist())
                    else:
                        pts.extend(pic[i][1][a:b+1,0].tolist())
                elif a>b:
                    pts.extend(pic[i][1][a:,0].tolist())
                    pts.extend(pic[i][1][:b+1,0].tolist())
                pts = np.array(pts).reshape((len(pts), 1, 2))
                ptsu.append(pts)
                cv2.polylines(img, [pts], True,(0,255,255),2)
            outer.append(ptsu)
            saveimg(img,path+r'\map\m%s.png'%(i))
        gama = 2.2
        (h255, w255) = np.where(add_im == 255)
        h255 = [int(i) for i in h255]
        w255 = [int(i) for i in w255]
        for i, j in zip(h255, w255):
            ds,dts = 0,[]
            for k in range(imagenum):
                if pic[k][0][i,j] == 255:
                    dists = [abs(cv2.pointPolygonTest(c,(j,i), True)) for c in outer[k]]
                    dists.sort()
                    d1,d2 = dists[0],dists[1]
                    if d1 == 0:
                        d1 = 1
                    if d2 == 0:
                        d2 = 1
                    dist = d1*d2
                    ds += dist
                    dts.append([k,dist])
            for d in dts:
                pic[d[0]][0][i,j] = round((math.sin((d[1]/ds-0.5)*math.pi)/2+0.5)**(1/gama)*255)
        if ad == 0:
            for i in range(len(pic)):
                mkdir(path + r'\map\%s\data' % (i))
                # saveimg(cv2.cvtColor(pic[i][0],cv2.COLOR_GRAY2BGR),path+r'\map\%s\0.png'%(i))
                saveimg(cv2.cvtColor(cv2.resize(pic[i][0],(0, 0),fx=1/fuseval.get(),fy=1/fuseval.get(),interpolation=cv2.INTER_AREA),cv2.COLOR_GRAY2BGR),path+r'\map\%s\0.png'%(i))
            clientwei, clienthei = clientweival.get(), clientheival.get()
            maplist = [os.path.join(i, 'cam1', 'cam1.csv') for i in path_list if os.path.exists(os.path.join(i, 'cam1', 'cam1.csv'))]
            with open(os.path.dirname(os.path.dirname(path)) + r'\20180423.xml', encoding="utf-8") as fp:
                xml = '<a>' + '\n' + fp.read() + '</a>'
            for i in range(imagenum):
                data = np.array(pd.read_csv(maplist[i], header=None))
                row, column = int(str(int(data[:,1][-1]))[:2])-1,int(str(int(data[:,1][-1]))[2:])-1
                calx = data[:, 4].reshape((row + 2, column + 2))
                caly = data[:, 5].reshape((row + 2, column + 2))
                calx, caly = interpolation(calx, caly,row,column, 1, 1, 1)  # 非网格节点插满
                calx = calx.reshape(((row + 2)*(column + 2), ))
                caly = caly.reshape(((row + 2)*(column + 2), ))
                data = data[:, 2:6]
                data[:, 0] = data[:, 0] / clientwei
                data[:, 1] = data[:, 1] / clienthei
                data[:, 2] = calx / (camwei / fuseval.get())
                data[:, 3] = caly / (camhei / fuseval.get())
                datas = np.zeros((data.shape[0], 3))
                datas = np.hstack((data, datas))
                np.savetxt(mkdir(path + r'\map\%s\data' % (i)) + r'\00texturKoord_0.txt', (datas), fmt='%.05f')

                xml_dom = parseString(xml)
                render = xml_dom.documentElement
                enable_log = render.getElementsByTagName("enable")[0].childNodes[0]  # 是否使用日志
                enable_log.data = '0'
                enable_blending = render.getElementsByTagName("enable_blending")[0].childNodes[0]  # 是否融合
                enable_blending.data = '0'
                width = render.getElementsByTagName("width")[0].childNodes[0]  # 分辨率宽
                width.data = str(clientwei)
                height = render.getElementsByTagName("height")[0].childNodes[0]  # 分辨率高
                height.data = str(clienthei)
                dim_x = render.getElementsByTagName("dim_x")[0].childNodes[0]  # 列
                dim_x.data = str(column+2)
                dim_y = render.getElementsByTagName("dim_y")[0].childNodes[0]  # 行
                dim_y.data = str(row+2)
                invert = render.getElementsByTagName("invert")[0].childNodes[0]  # 翻转
                invert.data = '0'
                start_number = render.getElementsByTagName("start_number")[0].childNodes[0]  # 起始帧
                start_number.data = '0'
                end_number = render.getElementsByTagName("end_number")[0].childNodes[0]  # 结束帧
                end_number.data = '0'
                image_source = render.getElementsByTagName("image_source")[0].childNodes[0]  # 输入路径
                image_source.data = r'F:\1\1\1\\'
                output_pattern = render.getElementsByTagName("output_pattern")[0].childNodes[0]  # 输出文件格式
                output_pattern.data = 'blending_%d.png'
                start = render.getElementsByTagName("start")[0].childNodes[0]  # 起始投影
                start.data = '0'
                end = render.getElementsByTagName("end")[0].childNodes[0]  # 结束投影
                end.data = '0'
                c0 = render.getElementsByTagName("c0")[0].childNodes[0]  # 输出路径
                c0.data = r'F:\0\0\0\\'
                xml_str = render.toxml(encoding='utf-8')
                xml_str = str(xml_str)
                xml_str = '\n'.join(xml_str.split(r'\n')[1:-1]) + '\n'
                xml_str = xml_str.replace(r'\t\t', '		').replace(r'\\\\', r'\\')
                xml_str = xml_str.replace(r'F:\\1\\1\\1', path + r'\map\%s' % (i))  # 输入路径
                xml_str = xml_str.replace(r'F:\\0\\0\\0', path_list[i] + r'\data')  # 输出路径
                with open(path + r"\map\%s\map.xml" % (i), "w") as f:
                    f.write(xml_str)

            for i in range(len(path_list)):
                t = threading.Thread(target=runxml, args=(path+r'\map\%s'%(i),path+r'\map\%s\map.xml'%(i),))
                t.setDaemon(True)
                t.start()
        else:
            for i in range(len(pic)):
                # saveimg(cv2.cvtColor(pic[i][0], cv2.COLOR_GRAY2BGR), path_list[i] + r'\0.png')
                saveimg(cv2.cvtColor(cv2.resize(pic[i][0],(0, 0),fx=1/fuseval.get(),fy=1/fuseval.get(),interpolation=cv2.INTER_AREA), cv2.COLOR_GRAY2BGR), path_list[i] + r'\0.png')
        showtext("生成融合模板完成！", 'black')
    except Exception as e:
        showtext(str(e.__traceback__.tb_lineno)+str(e), 'red')
# 运行xml文件
def runxml(path,file):
    os.chdir(path)
    os.system(file)
# 勾边融合
def edgefuse():
    maskfile = askopenfilename(title='选择勾边图片', filetype=[("勾边图片", "*.png")])
    try:
        if os.path.exists(maskfile):
            copyfi(maskfile,os.path.dirname(maskfile)+r'\0.png')
            mask = readimg(maskfile, 0)  # 打开的原始图片
            (hei,wei) = (mask.shape)
            df = pd.read_csv(os.path.dirname(maskfile)+r'\%s.csv'%(os.path.basename(os.path.dirname(maskfile))), header=None)
            df.columns = ['num', 'code', 'x', 'y', 'u', 'v']
            row, column = int(('00' + str(int(df['code'].iloc[-1])))[-4:-2]) - 1, int(
                ('00' + str(int(df['code'].iloc[-1])))[-2:]) - 1
            df1 = df.dropna(axis=0, subset=['u'])
            df2 = df[df[['u']].isnull().T.any()]
            df2 = interp2D(df1, df2, 'x', 'y', 'u', 'v','quintic')  # 二维插值
            df = df1.append(df2).sort_values(by='code')
            df.loc[:, 'u'], df.loc[:, 'v'] = df.loc[:, 'u'] / wei, df.loc[:, 'v'] / hei
            df.loc[:, 'x'], df.loc[:, 'y'] = df.loc[:, 'x'] / clientweival.get(), df.loc[:, 'y'] / clientheival.get()
            # bx = np.hstack((np.array(df[['u','v']]),np.array(df[['x','y']]),np.zeros(((column + 2)*(row + 2),3))))
            bx = np.hstack((np.array(df[['x','y']]),np.array(df[['u','v']]),np.zeros(((column + 2)*(row + 2),3))))
            np.savetxt(mkdir(os.path.dirname(maskfile) + r'\data')+r'\00texturKoord_0.txt', (bx), fmt='%.06f')
            with open(os.path.dirname(paths) + r'\20180423.xml', encoding="utf-8") as fp:
                xml = '<a>' + '\n' + fp.read() + '</a>'
            xml_dom = parseString(xml)
            render = xml_dom.documentElement
            enable_log = render.getElementsByTagName("enable")[0].childNodes[0]  # 是否使用日志
            enable_log.data = '0'
            enable_blending = render.getElementsByTagName("enable_blending")[0].childNodes[0]  # 是否融合
            enable_blending.data = '0'
            width = render.getElementsByTagName("width")[0].childNodes[0]  # 分辨率宽
            width.data = str(clientweival.get())
            height = render.getElementsByTagName("height")[0].childNodes[0]  # 分辨率高
            height.data = str(clientheival.get())
            dim_x = render.getElementsByTagName("dim_x")[0].childNodes[0]  # 列
            dim_x.data = str(column + 2)
            dim_y = render.getElementsByTagName("dim_y")[0].childNodes[0]  # 行
            dim_y.data = str(row + 2)
            invert = render.getElementsByTagName("invert")[0].childNodes[0]  # 0翻转,1不翻转
            invert.data = '1'
            start_number = render.getElementsByTagName("start_number")[0].childNodes[0]  # 起始帧
            start_number.data = '0'
            end_number = render.getElementsByTagName("end_number")[0].childNodes[0]  # 结束帧
            end_number.data = '0'
            image_source = render.getElementsByTagName("image_source")[0].childNodes[0]  # 输入路径
            image_source.data = r'F:\1\1\1\\'
            output_pattern = render.getElementsByTagName("output_pattern")[0].childNodes[0]  # 输出文件格式
            output_pattern.data = 'mb.png'
            start = render.getElementsByTagName("start")[0].childNodes[0]  # 起始投影
            start.data = '0'
            end = render.getElementsByTagName("end")[0].childNodes[0]  # 结束投影
            end.data = '0'
            c0 = render.getElementsByTagName("c0")[0].childNodes[0]  # 输出路径
            c0.data = r'F:\0\0\0\\'
            xml_str = render.toxml(encoding='utf-8')
            xml_str = str(xml_str)
            xml_str = '\n'.join(xml_str.split(r'\n')[1:-1])+'\n'
            xml_str = xml_str.replace(r'\t\t', '		').replace(r'\\\\', r'\\')
            xml_str = xml_str.replace(r'F:\\1\\1\\1', os.path.dirname(maskfile))  # 输入路径
            xml_str = xml_str.replace(r'F:\\0\\0\\0', os.path.dirname(maskfile))  # 输出路径
            with open(os.path.dirname(maskfile) + r"\mb.xml", "w") as f:
                f.write(xml_str)
            runxml(os.path.dirname(maskfile),os.path.dirname(maskfile) + r'\mb.xml')
    except Exception as e:
        showtext(str(e.__traceback__.tb_lineno)+str(e), 'red')

mousetype = 0 #0是空白模式,1是相机打开模式,1.5是正在拍照模式,2是图片打开模式,3是勾边模式,3.5是遮罩模式,4是勾边完成模式,5是蒙版填充模式
pointtype = 0 #0是未缩放模式,1是缩放模式,2是平移模式
current = None
firstpoint = [] #每个勾边的第一个点
firstkoint = [] #每个遮罩的第一个点
point = [] #勾边保存的点
koint = [] #遮罩保存的点
pointss = [] #勾边保存的总点
kointss = [] #遮罩保存的总点
linew = 1 #勾边线条宽度
markpoint = [] #mark点标记的点
coo = []
g_window_wh = [640, 480]  # 画布窗口宽高
g_location_win = [0, 0]  # 相对于大图，窗口在图片中的位置
location_win = [0, 0]  # 鼠标左键点击时，暂存相对窗口位置
g_location_click = [0, 0]  # 相对于窗口，鼠标左键点击和释放的位置
g_zoom, g_step = 0.25, 1.5  # 图片缩放比例和缩放系数
g_zoo = 0.25 #图片最小缩放比例

win=Tk()
camID1val = IntVar()#相机编号1
camID1val.set(1)
camID2val = IntVar()#相机编号2
camID2val.set(2)
camID3val = IntVar()#相机编号3
camID3val.set(3)
cammodel = StringVar() #相机型号
cammodel.set('IDS')
screen = IntVar() #默认银幕类型1是球幕2是环幕3是平幕
screen.set(1)
gainval = IntVar()#相机增益（0-100）
gainval.set(5)
exposureval = IntVar()#曝光时间（0.108ms-226.88ms）步长0.231
exposureval.set(30)
clientweival = IntVar()#客户端分辨率宽
clientweival.set(1920)
clientheival = IntVar()#客户端分辨率高
clientheival.set(1200)
camweival = IntVar()#相机分辨率宽
camweival.set(2560)
camheival = IntVar()#相机分辨率高
camheival.set(1920)
rowval = IntVar()#圆行数
rowval.set(25)
columnval = IntVar()#圆列数
columnval.set(40)
gridrow = IntVar() #网格行数
gridcolumn = IntVar() #网格列数
radiusval = IntVar()#圆半径
radiusval.set(18)
delayval = IntVar() #拍照延迟时间
delayval.set(3)
firstval = IntVar() #莱卡分列后起始点
firstval.set(3)
local_thval = IntVar() # 局部阈值
local_thval.set(25)
areaval = IntVar() # 最小识别圆面积
areaval.set(50)
marknumval = IntVar() #mark点标记序号
marknumval.set(0)
fixval = IntVar() #修正系数
fixval.set(15)
portval = IntVar() #添加客户端端口号
portval.set(10232)
addipval = StringVar() #添加客户端ip
projectval = StringVar() #添加场景名称
flyval = IntVar() # 判断飞点系数
fuseval = DoubleVar() #融合模板缩放倍数
fuseval.set(1.0)

vali = IntVar() #单，双，三相机
vali.set(1)
vali2 = IntVar() #局部分割
vali2.set(0)
vali4 = IntVar() #插值
vali4.set(1)

canvas = Canvas(win,bg='orange',highlightthickness=0)#绘制画布
canvas.place(width=640,height=480,relx=0.15625,rely=0.25)
canvas2 = Canvas(win,bg='violet',highlightthickness=0)#绘制画布
canvas3 = Canvas(win,bg='yellow',highlightthickness=0)#绘制画布
windnd.hook_dropfiles(canvas,func=run_openimg)
# 显示画布
def showcanvas():
    global canvas,canvas2,canvas3,pic_path
    canvas.place_forget()
    canvas2.place_forget()
    canvas3.place_forget()
    if cammodel.get() == "IDS":
        if vali.get() == 1:
            canvas.place(width=640, height=480, relx=0.15625, rely=0.25)
        elif vali.get() == 2:
            canvas.place(width=640, height=480, relx=0.15625, rely=0.25)
            canvas2.place(width=440, height=330, relx=0.65625, rely=0.25)
            canvas.delete('all')
        elif vali.get() == 3:
            canvas.place(width=640, height=480, relx=0.15625, rely=0.25)
            canvas2.place(width=320, height=240, relx=0.65625, rely=0.25)
            canvas3.place(width=320, height=240, relx=0.65625, rely=0.625)
            canvas.delete('all')
    elif cammodel.get() == "华睿":
        if vali.get() == 1:
            canvas.place(width=720, height=480, relx=0.15625, rely=0.25)
        elif vali.get() == 2:
            canvas.place(width=720, height=480, relx=0.15625, rely=0.25)
            canvas2.place(width=360, height=240, relx=0.71875, rely=0.25)
            canvas.delete('all')
        elif vali.get() == 3:
            canvas.place(width=720, height=480, relx=0.15625, rely=0.25)
            canvas2.place(width=360, height=240, relx=0.71875, rely=0.25)
            canvas3.place(width=360, height=240, relx=0.71875, rely=0.625)
            canvas.delete('all')
# 连接TCP
def conipport(ip,port):
    pk = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 创建socket对象
    pk.settimeout(1)
    result = pk.connect_ex((ip, int(port)))  # 建立客户端链接
    pk.close()
    return result
# 检测ip,端口是否连通
def checkcon():
    while True:
        try:
            for i in tree.get_children():
                for j in tree.get_children(i):
                    time.sleep(0.2)
                    ip, port = tree.item(j, 'values')[0][:-6],int(tree.item(j, 'values')[0][-5:])
                    sk = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 创建socket对象
                    sk.settimeout(0.5)
                    result = sk.connect_ex((ip, int(port)))  # 建立客户端链接
                    sk.close()
                    if result == 0:
                        color = 'blackfont'
                    else:
                        color = 'redfont'
                    tree.item(j,tags=color)
        except:
            pass
# 线程检测ip,端口是否连通
def run_checkcon():
    global check_ip
    try:
        check_ip = threading.Thread(target=checkcon)
        check_ip.setDaemon(True)
        check_ip.start()
    except Exception as e:
        showtext(str(e.__traceback__.tb_lineno)+str(e), 'red')
# 打开项目
def openconf():
    conf_path = askopenfilename(title='选择配置文件',filetype=[("", "*.ini")])
    readconf(conf_path,1)
# 读取配置文件
def readconf(conf_path,a):
    global projectname,iplist,portlist,camseriallist
    iplist, portlist = [], []
    if os.path.exists(conf_path):
        x = tree.get_children()
        for item in x:
            tree.delete(item)
        conf = configparser.ConfigParser()
        conf.read(conf_path, encoding="utf-8-sig")
        sections = conf.sections()
        # model = {0:'IDS',1:'华睿'}
        # for key,value in model.items():
        #     if key == conf.get('info', 'cammodel'):
        #         cammodel.set(value)
        cammodel.set(conf.get('info', 'cammodel')) # 相机品牌(IDS、华睿)
        camseriallist = conf.get('info', 'camserial').split(',') #华睿相机序列号列表
        screen.set(conf.get('info', 'screen')) # 默认银幕类型1是球幕2是环幕3是平幕
        gainval.set(conf.get('info', 'gain')) # 相机增益（0-100）
        exposureval.set(conf.get('info', 'exposure')) # 曝光时间（0.108ms-226.88ms）步长0.231
        clientweival.set(conf.get('info', 'clientwei')) # 客户端分辨率宽
        clientheival.set(conf.get('info', 'clienthei')) # 客户端分辨率高
        camweival.set(conf.get('info', 'camwei')) # 相机分辨率宽
        camheival.set(conf.get('info', 'camhei')) # 相机分辨率高
        rowval.set(conf.get('info', 'row')) # 圆行数
        columnval.set(conf.get('info', 'column')) # 圆列数
        radiusval.set(conf.get('info', 'radius')) # 圆半径
        delayval.set(conf.get('info', 'delay')) # 拍照延迟时间
        local_thval.set(conf.get('info', 'local')) # 局部阈值
        areaval.set(conf.get('info', 'area')) # 最小识别圆面积
        fixval.set(conf.get('info', 'fix')) # 修正系数
        vali.set(conf.get('info', 'camnum')) # 单，双，三相机
        vali2.set(conf.get('info', 'localuse')) # 局部分割
        vali4.set(conf.get('info', 'inter')) # 插值
        selectcammodel(0)
        showcanvas()
        projects = [i for i in sections if 'project' in i]
        for p in range(len(projects)):
            projectname = conf.get('project%s' % (p), 'name')
            iplist_ = conf.get('project%s' % (p), 'ip').split(',')
            portlist_ = conf.get('project%s' % (p), 'port').split(',')
            tree.insert('', p, 'sceen%s' % (p), values=projectname, text=projectname, open=True)
            for i, j in zip(iplist_, portlist_):
                # col = conipport(i, j)
                # if col == 0:
                #     color = 'blackfont'
                # else:
                #     color = 'redfont'
                color = 'blackfont'
                tree.insert('sceen%s' % (p), END, text=i.strip(' ') + '：' + j.strip(' '),
                            values=i.strip(' ') + ':' + j.strip(' '),tags=color)
                mkdir(paths + r'\%s\%s_%s' % (projectname, i.strip(' '), j.strip(' ')))
            iplist.extend(iplist_)
            portlist.extend(portlist_)
        if a == 0:
            run_checkcon()
# 保存项目
def saveconf():
    conf_path = asksaveasfilename(title='选择配置文件',filetype=[("", "*.ini")])
    if conf_path != '' and conf_path.isspace() is False and conf_path[-4:] != '.ini':
        conf_path = conf_path + '.ini'
    if conf_path[-4:] == '.ini':
        pname = tree.get_children()
        # conf = configparser.ConfigParser()
        conf = configparser.ConfigParser(allow_no_value=True)
        # conf.read(conf_path, encoding="utf-8-sig")
        conf.add_section("info")
        conf.set("info", "#相机品牌(IDS、华睿)", None)
        conf.set("info", "cammodel", str(cammodel.get()))
        a = ''
        for i in range(len(camseriallist)):
            a = a + camseriallist[i]
            if i != len(camseriallist)-1:
                a = a + ','
        conf.set("info", "#华睿相机序列号", None)
        conf.set("info", "camserial", a)
        conf.set("info", "#默认银幕类型1是球幕2是环幕3是平幕", None)
        screen.set(note.index('current')+1)
        conf.set("info", "screen", str(screen.get()))
        conf.set("info", "#相机增益（101-200）", None)
        conf.set("info", "gain", str(gainval.get()))
        conf.set("info", "#曝光时间（1ms-226ms）", None)
        conf.set("info", "exposure", str(exposureval.get()))
        conf.set("info", "#单屏分辨率宽", None)
        conf.set("info", "clientwei", str(clientweival.get()))
        conf.set("info", "#单屏分辨率高", None)
        conf.set("info", "clienthei", str(clientheival.get()))
        conf.set("info", "#相机分辨率宽", None)
        conf.set("info", "camwei", str(camweival.get()))
        conf.set("info", "#相机分辨率高", None)
        conf.set("info", "camhei", str(camheival.get()))
        conf.set("info", "#圆行数", None)
        conf.set("info", "row", str(rowval.get()))
        conf.set("info", "#圆列数", None)
        conf.set("info", "column", str(columnval.get()))
        conf.set("info", "#圆半径", None)
        conf.set("info", "radius", str(radiusval.get()))
        conf.set("info", "#拍照延迟时间", None)
        conf.set("info", "delay", str(delayval.get()))
        conf.set("info", "#局部阈值", None)
        conf.set("info", "local", str(local_thval.get()))
        conf.set("info", "#最小识别圆面积", None)
        conf.set("info", "area", str(areaval.get()))
        conf.set("info", "#修正系数", None)
        conf.set("info", "fix", str(fixval.get()))
        conf.set("info", "#默认相机数量", None)
        conf.set("info", "camnum", str(vali.get()))
        conf.set("info", "#默认是否局部分割打勾", None)
        conf.set("info", "localuse", str(vali2.get()))
        conf.set("info", "#默认是否插值打勾", None)
        conf.set("info", "inter", str(vali4.get()))
        conf.set("info", "#-------------------------------------------默认配置参数可以修改，格式请勿改动---------------------------------------------------------------------------------------------------------------", None)
        for i in range(len(pname)):
            conf.add_section("project%s"%(i))
            conf.set("project%s"%(i), "name",tree.item(pname[i],'values')[0])
            a,b = '',''
            for j in tree.get_children(pname[i]):
                a = a + tree.item(j,'values')[0][:-6]
                b = b + tree.item(j,'values')[0][-5:]
                if j != tree.get_children(pname[i])[-1]:
                    a,b = a + ',',b + ','
            conf.set("project%s"%(i), "ip",a)
            conf.set("project%s"%(i), "port",b)
        conf.write(open(conf_path, "w",encoding="utf-8-sig"))
# 选择相机型号
def selectcammodel(a):
    if cammodel.get() == "IDS":
        camweival.set(2560)
        camheival.set(1920)
    elif cammodel.get() == "华睿":
        camweival.set(3072)
        camheival.set(2048)
    showcanvas()
# 选择相机分辨率
def selectcamhei(a):
    c = [2560,3072,5472]
    b = [1920,2048,3648]
    for i in range(3):
        if c[i] == camweival.get():
            camheival.set(b[i])
# 选择屏幕分辨率
def selectclienthei(a):
    b = [1280, 1920, 4096]
    c = [800, 1200, 2160]
    d = [20,20,17]
    e = [32,32,32]
    f = [16,24,32]
    for i in range(3):
        if b[i] == clientweival.get():
            clientheival.set(c[i])
            rowval.set(d[i])
            columnval.set(e[i])
            radiusval.set(f[i])

win.title("相机网格服务器")     #添加窗口标题
# win.resizable(0,0) # 窗口宽高不可改
winmenu1 = Menu(win)
winmenu2 = Menu(winmenu1,tearoff=False)
winmenu1.add_cascade(label="文件",menu=winmenu2)
winmenu2.add_command(label='打开项目', command=openconf)
winmenu2.add_command(label='保存项目', command=saveconf)
winmenu3 = Menu(winmenu1,tearoff=False)
winmenu1.add_cascade(label="编辑",menu=winmenu3)
winmenu3.add_command(label='添加场景', command=addsceenwin)
winmenu3.add_command(label='删除场景', command=delsceen)
winmenu1.add_command(label="关于",command=explain)

win.configure(bg="#a7ea90",menu=winmenu1)        #窗口的背景颜色
winw,winh = 1280,640                          #窗口的宽高
scrw = win.winfo_screenwidth()            #屏幕的宽度
scrh = win.winfo_screenheight()           #屏幕的高度
winx = (scrw-winw)/2                        #窗口的水平位置
winy = (scrh-winh)/2-46                       #窗口的垂直位置
win.geometry("%dx%d+%d+%d" %(winw,winh,winx,winy))            #设置窗口位置
Label(text="",bg='white').place(relwidt=0.64375,relheight=0.25,relx=0.15625,rely=0)
def fixed_map(option):
    return [elm for elm in style.map("Treeview", query_opt=option) if elm[:2] != ("!disabled", "!selected")]
style = tk.Style()
style.map("Treeview",foreground=fixed_map("foreground"), background=fixed_map("background"))
style.configure("Treeview",font=("微软雅黑", 8),rowheight=14)
tree = tk.Treeview(win,selectmode='browse',show = "tree",style="Treeview")
tree.tag_configure('redfont',background="white",foreground='red')
tree.tag_configure('blackfont',background="white",foreground='black')
tree.place(relwidt=0.15625,relheight=0.25,relx=0,rely=0)
tree.bind("<<TreeviewSelect>>",selectPCip)
tree.bind("<Button-3>",conPc)
tree.bind("<Double-Button-1>",openclient)
# projectdate = datetime.datetime.now().strftime('%Y%m%d')[2:]
if os.path.exists(os.path.abspath('.')+r'\config.ini'):
    readconf(os.path.abspath('.')+r'\config.ini',0)
else:
    readconf('', 0)
#限制输入字符
def limitinput(a):
    if a.isdigit() or a == "":
        return True
    else:
        return False

limit = win.register(limitinput) #包装函数

lf1 = LabelFrame(win,text="相机参数",bg='white')
lf1.place(relwidt=0.15625,relheight=0.45,relx=0,rely=0.25)
Label(lf1,text="品牌",bg='white').place(relwidt=0.2,relheight=0.125,relx=0,rely=0)
cob0 = tk.Combobox(lf1,values=['IDS','华睿'],textvariable=cammodel)
cob0.bind("<<ComboboxSelected>>",selectcammodel)
cob0.place(relwidt=0.3,relheight=0.125,relx=0.2,rely=0)
bt0 = Button(lf1,text="打开相机",command=opencam,relief=GROOVE)
bt0.place(relwidt=0.5,relheight=0.125,relx=0.5,rely=0)
Label(lf1,text="数量",bg='white').place(relwidt=0.2,relheight=0.125,relx=0,rely=0.125)
rd1 = Radiobutton(lf1,variable=vali,value=1,text='1',command=showcanvas,bg='white')
rd1.place(relwidt=0.27,relheight=0.125,relx=0.2,rely=0.125)
rd2 = Radiobutton(lf1,variable=vali,value=2,text='2',command=showcanvas,bg='white')
rd2.place(relwidt=0.26,relheight=0.125,relx=0.47,rely=0.125)
rd3 = Radiobutton(lf1,variable=vali,value=3,text='3',command=showcanvas,bg='white')
rd3.place(relwidt=0.27,relheight=0.125,relx=0.73,rely=0.125)
Label(lf1,text="编号",bg='white').place(relwidt=0.2,relheight=0.125,relx=0,rely=0.25)
Entry(lf1,textvariable=camID1val,validate="key",validatecommand=(limit,'%P')).place(relwidt=0.27,relheight=0.125,relx=0.2,rely=0.25)
Entry(lf1,textvariable=camID2val,validate="key",validatecommand=(limit,'%P')).place(relwidt=0.26,relheight=0.125,relx=0.47,rely=0.25)
Entry(lf1,textvariable=camID3val,validate="key",validatecommand=(limit,'%P')).place(relwidt=0.27,relheight=0.125,relx=0.73,rely=0.25)
Scale(lf1, from_=101, to=200, resolution=1,orient=HORIZONTAL,bg='white',command=changegp,showvalue=0,variable=gainval,troughcolor="yellow").place(relwidt=1,relheight=0.125,relx=0,rely=0.375)
Label(lf1,text="增益",bg='white').place(relwidt=0.25,relheight=0.125,relx=0,rely=0.5)
Spinbox(lf1,from_=101,to=200,increment=1,textvariable=gainval,validate="key",validatecommand=(limit,'%P')).place(relwidt=0.25,relheight=0.125,relx=0.25,rely=0.5)
Label(lf1,text="曝光",bg='white').place(relwidt=0.25,relheight=0.125,relx=0.5,rely=0.5)
Spinbox(lf1,from_=1,to=226,increment= 1,textvariable=exposureval).place(relwidt=0.25,relheight=0.125,relx=0.75,rely=0.5)
Scale(lf1,from_=1,to=226,resolution=1,orient=HORIZONTAL,bg='white',command=changegp,showvalue=0,variable=exposureval,troughcolor="#22EBBB").place(relwidt=1,relheight=0.125,relx=0,rely=0.625)
Label(lf1,text="像素宽",bg='white').place(relwidt=0.22,relheight=0.125,relx=0,rely=0.75)
cob1 = tk.Combobox(lf1,values=[2560,3072,5472],textvariable=camweival,validate="key",validatecommand=(limit,'%P'))
cob1.bind('<<ComboboxSelected>>',selectcamhei)
cob1.place(relwidt=0.28,relheight=0.125,relx=0.22,rely=0.75)
Label(lf1,text="像素高",bg='white').place(relwidt=0.22,relheight=0.125,relx=0.5,rely=0.75)
tk.Combobox(lf1,values=[1920,2048,3648],textvariable=camheival,validate="key",validatecommand=(limit,'%P')).place(relwidt=0.28,relheight=0.125,relx=0.72,rely=0.75)
Label(lf1,text="拍照间隔",bg='white').place(relwidt=0.3,relheight=0.125,relx=0,rely=0.875)
Spinbox(lf1,from_=3,to=10,increment=1,textvariable=delayval,validate="key",validatecommand=(limit, '%P')).place(relwidt=0.2,relheight=0.125,relx=0.3,rely=0.875)
Button(lf1,text="银幕拍照",command=lambda:photo(0),relief=GROOVE).place(relwidt=0.5,relheight=0.125,relx=0.5,rely=0.875)

Label(text="单屏分辨率宽",bg='white').place(relwidt=0.1,relheight=0.05,relx=0,rely=0.7)
cob3 = tk.Combobox(win,values=[1280,1920,4096],textvariable=clientweival,validate="key",validatecommand=(limit,'%P'))
cob3.bind('<<ComboboxSelected>>',selectclienthei)
cob3.place(relwidt=0.05625,relheight=0.05,relx=0.1,rely=0.7)
Label(text="单屏分辨率高",bg='white').place(relwidt=0.1,relheight=0.05,relx=0,rely=0.75)
tk.Combobox(win,values=[800,1200,2160],textvariable=clientheival,validate="key",validatecommand=(limit,'%P')).place(relwidt=0.05625,relheight=0.05,relx=0.1,rely=0.75)

lf2 = LabelFrame(win,text="mark点",bg='white')
lf2.place(relwidt=0.15625,relheight=0.2,relx=0,rely=0.8)
Button(lf2,text="拍照",command=lambda:photo(1),relief=GROOVE).place(relwidt=0.5,relheight=0.3,relx=0,rely=0)
Button(lf2,text="识别",command=markdiscern,relief=GROOVE).place(relwidt=0.5,relheight=0.3,relx=0.5,rely=0)
Label(lf2,text="序号",bg='white').place(relwidt=0.5,relheight=0.3,relx=0,rely=0.35)
tk.Combobox(lf2,values=[i for i in range(17)],textvariable=marknumval,validate="key",validatecommand=(limit, '%P')).place(relwidt=0.5,relheight=0.3,relx=0.5,rely=0.35)
Label(lf2,text="leica",bg='white').place(relwidt=0.3,relheight=0.3,relx=0,rely=0.7)
Spinbox(lf2,from_=1,to=10,increment=1,textvariable=firstval,validate="key",validatecommand=(limit, '%P')).place(relwidt=0.2,relheight=0.3,relx=0.3,rely=0.7)
Button(lf2,text="分列",command=leica,relief=GROOVE).place(relwidt=0.5,relheight=0.3,relx=0.5,rely=0.7)

Button(win,text="选择目录",command=selectdirectory,relief=GROOVE).place(relwidt=0.08,relheight=0.05,relx=0.16,rely=0.01)
Label(text="行",bg='white').place(relwidt=0.02,relheight=0.05,relx=0.16,rely=0.07)
Entry(win,textvariable=rowval,validate="key",validatecommand=(limit, '%P')).place(relwidt=0.02,relheight=0.05,relx=0.18,rely=0.07)
Label(text="列",bg='white').place(relwidt=0.02,relheight=0.05,relx=0.2,rely=0.07)
Entry(win,textvariable=columnval,validate="key",validatecommand=(limit, '%P')).place(relwidt=0.02,relheight=0.05,relx=0.22,rely=0.07)
Label(text="半径",bg='white').place(relwidt=0.04,relheight=0.05,relx=0.16,rely=0.13)
tk.Combobox(win,values=[i for i in range(5,33)],textvariable=radiusval,validate="key",validatecommand=(limit, '%P')).place(relwidt=0.04,relheight=0.05,relx=0.2,rely=0.13)
Label(text="行",bg='white').place(relwidt=0.02,relheight=0.05,relx=0.16,rely=0.19)
Entry(win,textvariable=gridrow,validate="key",validatecommand=(limit, '%P')).place(relwidt=0.02,relheight=0.05,relx=0.18,rely=0.19)
Label(text="列",bg='white').place(relwidt=0.02,relheight=0.05,relx=0.2,rely=0.19)
Entry(win,textvariable=gridcolumn,validate="key",validatecommand=(limit, '%P')).place(relwidt=0.02,relheight=0.05,relx=0.22,rely=0.19)

Button(win,text="打开目录",command=opendirectory,relief=GROOVE).place(relwidt=0.08,relheight=0.05,relx=0.25,rely=0.01)
Button(win,text="显示标准点",command=lambda:run_showcircle(0),relief=GROOVE).place(relwidt=0.08,relheight=0.05,relx=0.25,rely=0.07)
Button(win,text="黑屏",command=lambda:run_showcircle(2),relief=GROOVE).place(relwidt=0.04,relheight=0.05,relx=0.25,rely=0.13)
Button(win,text="白屏",command=lambda:run_showcircle(3),relief=GROOVE).place(relwidt=0.03,relheight=0.05,relx=0.3,rely=0.13)
Button(win,text="显示网格点",command=lambda:run_showcircle(1),relief=GROOVE).place(relwidt=0.08,relheight=0.05,relx=0.25,rely=0.19)

Button(win,text="标准点拍照",command=lambda:seriesphoto(0),relief=GROOVE).place(relwidt=0.08,relheight=0.05,relx=0.34,rely=0.01)
Button(win,text="网格点拍照",command=lambda:seriesphoto(1),relief=GROOVE).place(relwidt=0.08,relheight=0.05,relx=0.34,rely=0.07)
Button(win,text="还原点拍照",command=lambda:seriesphoto(2),relief=GROOVE).place(relwidt=0.08,relheight=0.05,relx=0.34,rely=0.13)
# Button(win,text="精细拍照",command=finecam,relief=GROOVE).place(relwidt=0.08,relheight=0.05,relx=0.34,rely=0.19)

Button(win,text="标定还原",command=restorecalibration,relief=GROOVE).place(relwidt=0.08,relheight=0.05,relx=0.43,rely=0.01)
Button(win,text="勾边变形",command=edgefuse,relief=GROOVE).place(relwidt=0.08,relheight=0.05,relx=0.43,rely=0.07)

Label(text="飞点系数",bg='white').place(relwidt=0.055,relheight=0.05,relx=0.52,rely=0.01)
Spinbox(win,from_=1.4,to=7.0,increment=0.2,textvariable=flyval).place(relwidt=0.035,relheight=0.05,relx=0.575,rely=0.01)
Label(text="最小圆面积",bg='white').place(relwidt=0.055,relheight=0.05,relx=0.52,rely=0.07)
tk.Combobox(win,values=[i for i in range(50,1050,50)],textvariable=areaval,validate="key",validatecommand=(limit, '%P')).place(relwidt=0.035,relheight=0.05,relx=0.575,rely=0.07)
Checkbutton(win,variable=vali2,text="局部分割",bg='white').place(relwidt=0.055,relheight=0.05,relx=0.52,rely=0.13)
tk.Combobox(win,values=[i for i in range(3,257,2)],textvariable=local_thval,validate="key",validatecommand=(limit, '%P')).place(relwidt=0.035,relheight=0.05,relx=0.575,rely=0.13)
Checkbutton(win,variable=vali4,text="插值",bg='white').place(relwidt=0.04,relheight=0.05,relx=0.52,rely=0.19)
Button(win,text="单独插值",command=interp,relief=GROOVE).place(relwidt=0.05,relheight=0.05,relx=0.56,rely=0.19)

pro = tk.Progressbar(win, mode="determinate", value=0, max=100, length=200)
pro.place(relwidt=0.2,relheight=0.02,relx=0.8,rely=0)

scr1 = Scrollbar(win)
text = Text(win,bg='white',yscrollcommand=scr1.set)
text.place(relwidt=0.185,relheight=0.23,relx=0.8,rely=0.02)
scr1.place(relwidt=0.015,relheight=0.23,relx=0.985,rely=0.02)
scr1.config(command=text.yview)

canvas.bind("<Motion>", mousemove) #鼠标移动
canvas.bind("<Button-1>", leftdown) #鼠标左键点点击
canvas.bind("<Button-2>", middledown) #鼠标中键点点击
canvas.bind("<Button-3>", rightdown) #鼠标右键点点击
canvas.bind("<MouseWheel>", wheel) #鼠标滚轮
canvas.bind("<Double-Button-1>", mousedouble) #鼠标左键双击
canvas.bind("<B2-Motion>", downmove) #按下鼠标中键移动
canvas.bind("<space>", fillmask) #点击空格键,保存蒙版
canvas.bind("<Return>", cammask) #点击回车键,保存相机蒙版
canvas.bind("<Control-q>", openimg) #按下Ctrl+q打开图片
canvas.bind("<Control-x>", closeimg) #按下Ctrl+x关闭图片
canvas.bind("<Control-a>", newedge) #按下Ctrl+a新建勾边
canvas.bind("<Control-s>", newshade) #按下Ctrl+s添加遮罩
canvas.bind("<Control-w>", openmask) #按下Ctrl+w相机勾边
canvas.bind("<Control-i>", autoedge) #按下Ctrl+i自动勾边
canvas.bind("<Control-z>", delpoint) #按下Ctrl+z撤销上个点
# canvas.bind("<Escape>", closecam) #点击Esc键关闭相机

note = tk.Notebook(win)   #添加选项卡容器
note.place(relwidt=0.18,relheight=0.25,relx=0.62,rely=0)   #添加选项卡容器

pane1 = Frame(bg='white')
# pane1.tag_configure('redfont',background="white",foreground='red')
pane2 = Frame(bg='white')
pane3 = Frame(bg='white')
note.add(pane1, text=" 球 幕 ")   #添加第一个选项卡
note.add(pane2, text=" 环 幕 ")   #添加第二个选项卡
note.add(pane3, text=" 平 幕 ")   #添加第三个选项卡

if screen.get() == 1: #球幕
    note.select(pane1)
    flyval.set(5)
    gridrow.set(37)
    gridcolumn.set(37)
elif screen.get() == 2: #环幕
    note.select(pane2)
    flyval.set(2.5)
else: #平幕
    note.select(pane3)
    flyval.set(2)

def selectscreen(): #选择银幕标签
    if note.index('current')+1 == 1: #球幕
        flyval.set(5)
        gridrow.set(37)
        gridcolumn.set(37)
    elif note.index('current')+1 == 2: #环幕
        flyval.set(2.5)
    elif note.index('current')+1 == 3: #平幕
        flyval.set(2)

selectscreen()
note.bind('<<NotebookTabChanged>>',lambda event:selectscreen())
Button(pane1,text="标准点识别",command=lambda:run_discern(0,0),relief=GROOVE).place(relwidt=0.48,relheight=0.25,relx=0.02,rely=0.15)
Button(pane1,text="网格点识别",command=lambda:run_discern(1,0),relief=GROOVE).place(relwidt=0.48,relheight=0.25,relx=0.02,rely=0.45)
Button(pane1,text="还原点识别",command=lambda:run_discern(2,0),relief=GROOVE).place(relwidt=0.48,relheight=0.25,relx=0.02,rely=0.75)
Spinbox(pane1,from_=-99,to=99,increment=1,textvariable=fixval,validate="key",validatecommand=(limit,'%P')).place(relwidt=0.18,relheight=0.25,relx=0.52,rely=0.15)
Button(pane1,text="自动修正",command=lambda:run_autofix(0),relief=GROOVE).place(relwidt=0.3,relheight=0.25,relx=0.7,rely=0.15)
Spinbox(pane1,from_=0.25,to=1,increment=0.05,textvariable=fuseval).place(relwidt=0.18,relheight=0.25,relx=0.52,rely=0.45)
Button(pane1,text="生成融合",command=run_domefuse,relief=GROOVE).place(relwidt=0.3,relheight=0.25,relx=0.7,rely=0.45)
Button(pane1,text="网格还原",command=lambda:run_restoregrid(0,0),relief=GROOVE).place(relwidt=0.48,relheight=0.25,relx=0.52,rely=0.75)

Button(pane2,text="标准点识别",command=lambda:run_discern(0,1),relief=GROOVE).place(relwidt=0.48,relheight=0.25,relx=0.02,rely=0.15)
Button(pane2,text="网格点识别",command=lambda:run_discern(1,1),relief=GROOVE).place(relwidt=0.48,relheight=0.25,relx=0.02,rely=0.45)
Button(pane2,text="还原点识别",command=lambda:run_discern(2,1),relief=GROOVE).place(relwidt=0.48,relheight=0.25,relx=0.02,rely=0.75)
Spinbox(pane2,from_=-99,to=99,increment=1,textvariable=fixval,validate="key",validatecommand=(limit,'%P')).place(relwidt=0.18,relheight=0.25,relx=0.52,rely=0.15)
Button(pane2,text="自动修正",command=lambda:run_autofix(1),relief=GROOVE).place(relwidt=0.3,relheight=0.25,relx=0.7,rely=0.15)
Button(pane2,text="网格计算",command=lambda:run_restoregrid(1,1),relief=GROOVE).place(relwidt=0.48,relheight=0.25,relx=0.52,rely=0.45)
Button(pane2,text="网格还原",command=lambda:run_restoregrid(1,0),relief=GROOVE).place(relwidt=0.48,relheight=0.25,relx=0.52,rely=0.75)

Button(pane3,text="标准点识别",command=lambda:run_discern(0,2),relief=GROOVE).place(relwidt=0.48,relheight=0.25,relx=0.02,rely=0.15)
Button(pane3,text="网格点识别",command=lambda:run_discern(1,2),relief=GROOVE).place(relwidt=0.48,relheight=0.25,relx=0.02,rely=0.45)
Button(pane3,text="还原点识别",command=lambda:run_discern(2,2),relief=GROOVE).place(relwidt=0.48,relheight=0.25,relx=0.02,rely=0.75)
Spinbox(pane3,from_=-99,to=99,increment=1,textvariable=fixval,validate="key",validatecommand=(limit,'%P')).place(relwidt=0.18,relheight=0.25,relx=0.52,rely=0.15)
Button(pane3,text="自动修正",command=lambda:run_autofix(2),relief=GROOVE).place(relwidt=0.3,relheight=0.25,relx=0.7,rely=0.15)
Button(pane3,text="网格计算",command=lambda:run_restoregrid(2,1),relief=GROOVE).place(relwidt=0.48,relheight=0.25,relx=0.52,rely=0.45)
Button(pane3,text="网格还原",command=lambda:run_restoregrid(2,0),relief=GROOVE).place(relwidt=0.48,relheight=0.25,relx=0.52,rely=0.75)
win.iconbitmap(resource_path('0.ico')) #添加窗口图标

win.mainloop()