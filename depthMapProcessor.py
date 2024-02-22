import numpy as np
import cv2
import time

class DepthMapProcessor:
    #Load camera parameters
    # ret = np.load('param_ret.npy')
    # K = np.load('param_K.npy')
    # dist = np.load('param_dist.npy')

    #cap = cv2.VideoCapture("/home/sieuwe/Desktop/vidz/full_2.avi")
    #cap = cv2.VideoCapture("/home/sieuwe/Desktop/vidz/full_3.avi")
    #cap = cv2.VideoCapture("/home/sieuwe/Desktop/vidz/full_1.avi")
    #cap = cv2.VideoCapture(2)
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3448)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 808)

    kernel= np.ones((13,13),np.uint8)

    #Stereo matcher settings
    win_size = 5
    min_disp = 10
    max_disp = 16 * 2 + 10
    num_disp = max_disp - min_disp # Needs to be divisible by 16
    h,w = (800, 1264)

    def __init__(self,param_ret,param_k,param_dist):
        self.ret = param_ret
        self.K = param_k
        self.dist = param_dist
        
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(self.K,self.dist,(self.w,self.h),1,(self.w,self.h))

        self.mapx, self.mapy = cv2.initUndistortRectifyMap(self.K,self.dist, None ,new_camera_matrix,(self.w, self.h),cv2.CV_16SC2)

        self.stereo = cv2.StereoSGBM_create(minDisparity= self.min_disp,
        numDisparities =  self.num_disp,
        blockSize = 5,
        uniquenessRatio = 10,
        speckleWindowSize = 1000,
        speckleRange = 10,
        disp12MaxDiff = 25,
        P1 = 8*3* self.win_size**2,#8*3*win_size**2,
        P2 =32*3* self.win_size**2) #32*3*win_size**2)

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

    def processFrame(self,frameL,frameR):

        # ret, frame = cap.read()

        start = time.time()

        # frameR, frameL = decode(frame)

        #Undistort images
        #img_1_undistorted = cv2.undistort(frameL, K, dist, None, new_camera_matrix)
        #img_2_undistorted = cv2.undistort(frameR, K, dist, None, new_camera_matrix)
        #img_1_undistorted= cv2.remap(frameL,mapx,mapy, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)  
        #img_2_undistorted= cv2.remap(frameR,mapx,mapy, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

        #downsample image for higher speed
        frameL_downsampled = cv2.pyrDown(cv2.cvtColor(frameL, cv2.COLOR_BGR2GRAY))
        frameR_downsampled = cv2.pyrDown(cv2.cvtColor(frameR, cv2.COLOR_BGR2GRAY))

        # img_1_downsampled = cv2.cvtColor(frameL, cv2.COLOR_BGR2GRAY)
        # img_2_downsampled = cv2.cvtColor(frameR, cv2.COLOR_BGR2GRAY)

        new_w, new_h = frameL_downsampled.shape

        #compute stereo
        disp = self.stereo.compute(frameL_downsampled,frameR_downsampled)
        
        #denoise step 1
        denoised = ((disp.astype(np.float32)/ 16)- self.min_disp)/ self.num_disp
        dispc= (denoised-denoised.min())*255
        dispC= dispc.astype(np.uint8)     
        
        #denoise step 2
        denoised= cv2.morphologyEx(dispC,cv2.MORPH_CLOSE,  self.kernel)
        
        #apply color map
        disp_Color= cv2.applyColorMap(denoised,cv2.COLORMAP_OCEAN)
        
        f = 0.3*self.w                          # 30cm focal length
        Q = np.float32([[1, 0, 0, -0.5*new_w],
                        [0,-1, 0,  0.5*new_h],  # turn points 180 deg around x-axis,
                        [0, 0, 0,      f],      # so that y-axis looks up
                        [0, 0, 1,      0]])
        points = cv2.reprojectImageTo3D(disp, Q)

        print(np.shape(points))

        # z_values = points[:,:,2]
        # z_values = z_values.flatten()
        # indices = z_values.argsort()

        # precentage = 25280
        # min_distance = np.mean(np.take(z_values,indices[0:precentage]))                             # takes the 30% lowest measuerements and gets the average distance from these.
        # avg_distance = np.mean(z_values)                                                           # averages all distances
        # max_distance = np.mean(np.take(z_values,indices[z_values.shape[0]-precentage:z_values.shape[0]])) # takes the 30% highest measuerements and gets the average distance from these.
        #print(np.take(z_values,indices[z_values.shape[0]-precentage:z_values.shape[0]]))
        #print(np.take(z_values,indices[:-100]))

        #visualize
        # cv2.imshow("Depth", disp_Color)
        frameL = cv2.resize(frameL,(632, 400))
        color_depth = cv2.addWeighted(frameL,0.4,disp_Color,0.4,0)

        end = time.time()
        fps = 1 / (end-start)

        # cv2.putText(color_depth, "minimum: " + str(round(min_distance,1)),(5, 20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2,cv2.LINE_AA)
        # cv2.putText(color_depth, "average: " + str(round(avg_distance,1)),(5, 40),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2,cv2.LINE_AA)
        # cv2.putText(color_depth, "maximum: " + str(round(max_distance,1)),(5, 60),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2,cv2.LINE_AA)
        cv2.putText(color_depth, "FPS: " + str(round(fps)),(5, 80),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2,cv2.LINE_AA)

        # cv2.imshow("color & Depth", color_depth)
        return disp_Color,color_depth,points

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