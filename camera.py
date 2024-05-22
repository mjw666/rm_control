# coding:utf-8
import os
import cv2
import glob
import time
import numpy as np
from pathlib import Path
from utils.augmentations import letterbox
from threading import Thread
from utils.general import clean_str


img_formats = ['bmp', 'jpg', 'jpeg', 'png', 'tif', 'tiff', 'dng', 'webp']  # acceptable image suffixes
vid_formats = ['mov', 'avi', 'mp4', 'mpg', 'mpeg', 'm4v', 'wmv', 'mkv']  # acceptable video suffixes


class LoadImages:  # for inference
    def __init__(self, path, img_size=640, stride=32):
        p = str(Path(path).absolute())  # os-agnostic absolute path
        if '*' in p:
            files = sorted(glob.glob(p, recursive=True))  # glob
        elif os.path.isdir(p):
            files = sorted(glob.glob(os.path.join(p, '*.*')))  # dir
        elif os.path.isfile(p):
            files = [p]  # files
        else:
            raise Exception(f'ERROR: {p} does not exist')

        images = [x for x in files if x.split('.')[-1].lower() in img_formats]
        videos = [x for x in files if x.split('.')[-1].lower() in vid_formats]
        ni, nv = len(images), len(videos)

        self.img_size = img_size
        self.stride = stride
        self.files = images + videos
        self.nf = ni + nv  # number of files
        self.video_flag = [False] * ni + [True] * nv
        self.mode = 'image'
        if any(videos):
            self.new_video(videos[0])  # new video
        else:
            self.cap = None
        assert self.nf > 0, f'No images or videos found in {p}. ' \
                            f'Supported formats are:\nimages: {img_formats}\nvideos: {vid_formats}'

    def __iter__(self):
        self.count = 0
        return self

    def __next__(self):
        if self.count == self.nf:
            raise StopIteration
        path = self.files[self.count]

        if self.video_flag[self.count]:
            # Read video
            self.mode = 'video'
            ret_val, img0 = self.cap.read()
            if not ret_val:
                self.count += 1
                self.cap.release()
                if self.count == self.nf:  # last video
                    raise StopIteration
                else:
                    path = self.files[self.count]
                    self.new_video(path)
                    ret_val, img0 = self.cap.read()

            self.frame += 1
            print(f'video {self.count + 1}/{self.nf} ({self.frame}/{self.nframes}) {path}: ', end='')

        else:
            # Read image
            self.count += 1
            img0 = cv2.imread(path)  # BGR
            assert img0 is not None, 'Image Not Found ' + path
            print(f'image {self.count}/{self.nf} {path}: ', end='')

        # Padded resize
        img = letterbox(img0, self.img_size, stride=self.stride)[0]

        # Convert
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)

        return path, img, img0, self.cap

    def new_video(self, path):
        self.frame = 0
        self.cap = cv2.VideoCapture(path)
        self.nframes = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))

    def __len__(self):
        return self.nf  # number of files


class LoadWebcam:  # for inference
    def __init__(self, pipe='0', img_size=(384, 640), stride=16):
        self.img_size = img_size
        self.stride = stride

        if pipe.isnumeric():
            pipe = eval(pipe)  # local camera
        # pipe = 'rtsp://192.168.1.64/1'  # IP camera
        # pipe = 'rtsp://username:password@192.168.1.64/1'  # IP camera with login
        # pipe = 'http://wmccpinetop.axiscam.net/mjpg/video.mjpg'  # IP golf camera

        self.pipe = pipe
        self.cap = cv2.VideoCapture(pipe)  # video capture object
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)  # set buffer size

    def __iter__(self):
        self.count = -1
        return self

    def __next__(self):
        self.count += 1
        if cv2.waitKey(1) == ord('q'):  # q to quit
            self.cap.release()
            cv2.destroyAllWindows()
            raise StopIteration

        # Read frame
        if self.pipe == 0:  # local camera
            ret_val, img0 = self.cap.read()
            img0 = cv2.flip(img0, 1)  # flip left-right
        else:  # IP camera
            n = 0
            while True:
                n += 1
                self.cap.grab()
                if n % 30 == 0:  # skip frames
                    ret_val, img0 = self.cap.retrieve()
                    if ret_val:
                        break

        # Print
        assert ret_val, f'Camera Error {self.pipe}'
        img_path = 'webcam.jpg'
        print(f'webcam {self.count}: ', end='')

        # Padded resize
        img = letterbox(img0, self.img_size, stride=self.stride)[0]

        # Convert
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)

        return img_path, img, img0, None

    def __len__(self):
        return 0


class LoadStreams:  # multiple IP or RTSP cameras
    def __init__(self, sources='streams.txt', img_size=640, stride=32):
        self.mode = 'stream'
        self.img_size = img_size
        self.stride = stride

        if os.path.isfile(sources):
            with open(sources, 'r') as f:
                sources = [x.strip() for x in f.read().strip().splitlines() if len(x.strip())]
        else:
            sources = [sources]

        n = len(sources)
        self.imgs = [None] * n
        self.sources = [clean_str(x) for x in sources]  # clean source names for later
        for i, s in enumerate(sources):
            # Start the thread to read frames from the video stream
            print(f'{i + 1}/{n}: {s}... ', end='')
            #cap = cv2.VideoCapture(eval(s) if s.isnumeric() else s)
            ## assert cap.isOpened(), f'Failed to open {s}'
            #w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            #h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            #fps = cap.get(cv2.CAP_PROP_FPS) % 100
            fps = 30
            #_, self.imgs[i] = cap.read()  # guarantee first frame
            thread = Thread(target=self.update, args=([i, s, fps]), daemon=True)
            #print(f' success ({w}x{h} at {fps:.2f} FPS).')
            thread.start()
        print('')  # newline

        # check for common shapes
        self.rect=True
        #try:
        #    s = np.stack([letterbox(x, self.img_size, stride=self.stride)[0].shape for x in self.imgs], 0)  # shapes
        #    print('========shape===========:', s)
        #    self.rect = np.unique(s, axis=0).shape[0] == 1  # rect inference if all shapes equal
        #    if not self.rect:
        #        print('WARNING: Different stream shapes detected. For optimal performance supply similarly-shaped streams.')
        #except:
        #    print('cccccccccc')

    def update(self, index, s, fps):
        # Read next stream frame in a daemon thread
        cap = cv2.VideoCapture(eval(s) if s.isnumeric() else s)
        #assert cap.isOpened(), f'Failed to open {s}'
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f' success ({w}x{h} at {fps:.2f} FPS).')
        _, self.imgs[index] = cap.read()  # guarantee first frame
        n = 0
        status = 1
        #while cap.isOpened():
        while True:
            n += 1
            # _, self.imgs[index] = cap.read()
            ret = cap.grab()
            if n == int(100 / fps):  # read every 4th frame
                success, im = cap.retrieve()
                if not success:
                  type(im)
                  with open('streams.txt', 'r') as f:
                      sources = [x.strip() for x in f.read().strip().splitlines() if len(x.strip())]
                      s = sources[index]
                      #print('retry video cap ............................................................................')
                      #print(s)
                      try:
                         cap = cv2.VideoCapture(eval(s) if s.isnumeric() else s)
                         print(cap.isOpened())
                      except:
                         print('xxx')
                      time.sleep(1)
                      n = 0
                      continue
                elif not self.rect:
                  s = np.stack([letterbox(x, self.img_size, stride=self.stride)[0].shape for x in self.imgs], 0)  # shapes
                  self.rect = np.unique(s, axis=0).shape[0] == 1  # rect inference if all shapes equal
                  if not self.rect:
                     print('WARNING: Different stream shapes detected. For optimal performance supply similarly-shaped streams.')
                #else:
                #    print('rect---------', self.rect)
                #    #pass
                if im is None:
                    print("im is None")
                self.imgs[index] = im if success else self.imgs[index] * 0
                n = 0
            #time.sleep(1 / fps)  # wait time
            time.sleep(0.01)

    def __iter__(self):
        self.count = -1
        return self

    def __next__(self):
        self.count += 1
        img0 = self.imgs.copy()
        #if cv2.waitKey(1) == ord('q'):  # q to quit
        #    cv2.destroyAllWindows()
        #    raise StopIteration

        # Letterbox
        #img = [letterbox(x, self.img_size, auto=self.rect, stride=self.stride)[0] for x in img0 if x is not None]
        #img = [letterbox(x, self.img_size, auto=self.rect, stride=self.stride)[0] for x in img0]
        
        img = []
        for index, item in enumerate(img0):
            if item is None:
                # TODO:造一串全0的假数据
                   
                #print("device index is none",index)
                box_value = np.zeros((480,640,3),dtype=float)
                
            else:
                box_value = letterbox(item, self.img_size, auto=self.rect, stride=self.stride)[0]
                #print('type', type(box_value), 'shape', box_value.shape)
            img.append(box_value)
       


        #if len(img) == 0 :
        #    return self.sources, None, img0, None
        # Stack
        img = np.stack(img, 0)

        # Convert
        img = img[:, :, :, ::-1].transpose(0, 3, 1, 2)  # BGR to RGB, to bsx3x416x416
        img = np.ascontiguousarray(img)

        return self.sources, img, img0, None

    def __len__(self):
        return 0  # 1E12 frames = 32 streams at 30 FPS for 30 years


