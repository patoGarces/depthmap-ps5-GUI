import numpy as np
import cv2
import time

from scipy import stats

class DepthMapProcessor:
    #Load camera parameters
    # ret = np.load('param_ret.npy')
    # K = np.load('param_K.npy')
    # dist = np.load('param_dist.npy')

    #cap = cv2.VideoCapture("/home/sieuwe/Desktop/vidz/full_2.avi")
    #cap = cv2.VideoCapture("/home/sieuwe/Desktop/vidz/full_3.avi")
    #cap = cv2.VideoCapture("/home/sieuwe/Desktop/vidz/full_1.avi")

    kernel= np.ones((13,13),np.uint8)

    #Stereo matcher settings
    min_disp = 10
    max_disp = 16 * 2 + 10
    num_disp = max_disp - min_disp # Needs to be divisible by 16
    h,w = (800, 1264)

    def __init__(self,param_ret,param_k,param_dist):
        self.ret = param_ret
        self.K = param_k
        self.dist = param_dist

        self.mixerDepth = 0.5
        
        self.new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(self.K,self.dist,(self.w,self.h),1,(self.w,self.h))

        self.mapx, self.mapy = cv2.initUndistortRectifyMap(self.K,self.dist, None ,self.new_camera_matrix,(self.w, self.h),cv2.CV_16SC2)
        self.setStereoSettings()

    def setStereoSettings(self,_blockSize = 5,_winSize = 5):
        
        self.stereo = cv2.StereoSGBM_create(
            minDisparity= self.min_disp,
            numDisparities =  self.num_disp,
            blockSize = _blockSize,
            uniquenessRatio = 10,
            speckleWindowSize = 1000,
            speckleRange = 10,
            disp12MaxDiff = 25,
            P1 =     8*3* _winSize**2,
            P2 =    32*3* _winSize**2
        )

    def changeMixDepthValue(self,_mixValue):
        self.mixerDepth = _mixValue

    def changeBlockSizeValue(self,_blockSize = 5):
        self.setStereoSettings(_blockSize = _blockSize)

    def changeWinSizeValue(self,_winSize = 5):
        try:
            self.setStereoSettings(_winSize = _winSize)
        except Exception as error:
            print("Error: ",error)

    def decode(self,frame):
        height, width, _ = frame.shape
        mid = width // 2
        left_frame = frame[:, :mid, :]
        right_frame = frame[:, mid:, :]

        left_frame = cv2.resize(left_frame,(1264,800))
        right_frame = cv2.resize(right_frame,(1264,800))
        
        return (left_frame, right_frame)

    def get_distance(d):
        return 30 * 10/d

    def postProcess(self,disp_matrix):                                  # TODO: mover de aca, solo para prueba de filtrado

        pp_disp = np.copy(disp_matrix)

        for x in range(pp_disp.shape[1]):
            for y in range(pp_disp.shape[0]):

                # MEAN
                avg = np.mean(pp_disp[y-7:y+8, x-7:x+8]) 
                if np.absolute(pp_disp[y, x] - avg) > 5:
                    pp_disp[y, x] = avg

                # MODE
                if x > 12 and x < (pp_disp.shape[0] - 12):
                    if pp_disp[y, x] > 25:
                        mode = stats.mode(pp_disp[y-12:y+13, x-12:x+13].flatten())
                        pp_disp[y, x] = mode[0][0]

                # THRESHOLD
                if pp_disp[y, x] > 30:
                    pp_disp[y, x] = 25

        print(f"Post-processing for disparity matrix of shape {disp_matrix.shape} complete.")

        return pp_disp
    
    def rectifiedFrames(self,frameL,frameR): 
        #Undistort images
        frameL = cv2.undistort(frameL, self.K, self.dist, None, self.new_camera_matrix)
        frameR = cv2.undistort(frameR, self.K, self.dist, None, self.new_camera_matrix)
        frameL= cv2.remap(frameL,self.mapx,self.mapy, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)  
        frameR= cv2.remap(frameR,self.mapx,self.mapy, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

        return frameL, frameR
    
    def downsampledFrames(self,frameL,frameR):
        # downsample image for higher speed
        frameL_downsampled = cv2.pyrDown(frameL)
        frameR_downsampled = cv2.pyrDown(frameR)

        return frameL_downsampled,frameR_downsampled
    
    def getHorizontalStripe(self,frameL,frameR):
        # Corte para una sola franja:

        print('getHorizontalStripe: ' + str(np.shape(frameL)))
        height, width, _  = frameL.shape
        y_start = int(height / 2) - 10  # Comienza 10 píxeles por encima del centro
        y_end = y_start + 20  # Toma 20 píxeles de alto
        frameLCut = frameL[y_start:y_end, :]
        frameRCut =  frameR[y_start:y_end, :]

        return frameLCut,frameRCut


    def getStereoDepthmap(self,frameL,frameR):
        start = time.time()
        
        frameLGray = cv2.cvtColor(frameL, cv2.COLOR_BGR2GRAY)
        frameRGray = cv2.cvtColor(frameR, cv2.COLOR_BGR2GRAY)

        frameRgbL = frameL

        #compute stereo
        disp = self.stereo.compute(frameLGray,frameRGray)
        
        #denoise step 1
        denoised = ((disp.astype(np.float32)/ 16)- self.min_disp)/ self.num_disp
        dispc= (denoised-denoised.min())*255
        dispC= dispc.astype(np.uint8)     
        
        #denoise step 2
        denoised= cv2.morphologyEx(dispC,cv2.MORPH_CLOSE,  self.kernel)
        
        #apply color map
        disp_color= cv2.applyColorMap(denoised,cv2.COLORMAP_JET)#cv2.COLORMAP_OCEAN)
        
        new_h,new_w = frameLGray.shape

        f = 0.3*self.w                          # 30cm focal length
        Q = np.float32([[1, 0, 0, -0.5*new_w],
                        [0,-1, 0,  0.5*new_h],  # turn points 180 deg around x-axis,
                        [0, 0, 0,      -f],      # so that y-axis looks up
                        [0, 0, 1,      0]])
        
        # Qy = np.float32([   [1, 0, 0, 0],
        #                     [0, -1, 0, 0],  # turn points 180 deg around x-axis,
        #                     [0, 0, 1, 0],      # so that y-axis looks up
        #                     [0, 0, 0, 1]])
        
        # Q = np.dot(Q,Qy)
        
        points = cv2.reprojectImageTo3D(disp, Q)
        # points = cv2.reprojectImageTo3D(denoised, Q)

        color_depth = cv2.addWeighted(frameRgbL,1-self.mixerDepth,disp_color,self.mixerDepth,0)

        end = time.time()
        fps = 1 / (end-start)

        cv2.putText(color_depth, "FPS: " + str(round(fps)),(5, 80),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2,cv2.LINE_AA)

        return color_depth,points,disp,disp_color,denoised

# ret = np.load('DepthParams/param_ret.npy')
# K = np.load('DepthParams/param_K.npy')
# dist = np.load('DepthParams/param_dist.npy')

# depthMapProcessor = DepthMapProcessor(ret,K,dist)
# cap = cv2.VideoCapture(1)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
# cap.set(cv2.CAP_PROP_FPS, 60)

# while(True):
#     ret, frame = cap.read()
#     frameR, frameL = depthMapProcessor.decode(frame)

#     depth,disp_Color = depthMapProcessor.processFrame(frameL,frameR)
#     cv2.imshow("Depth", disp_Color)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#             break