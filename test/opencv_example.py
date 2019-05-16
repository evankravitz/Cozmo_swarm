import asyncio
import cozmo
import PIL.Image
import PIL.ImageFont
import PIL.ImageTk
import cv2 as cv
import scipy.misc
#from scipy.interpolate import UnivariateSpline
import numpy as np
import tkinter as tk
import sys
import time

class Thresh:
    def __init__(self):
        self._robot = None
        self._tk_root = 0
        self._tk_label_input = 0
        self._tk_label_output = 0
        cozmo.connect(self.run)
        
    def on_img(self, event, *, image:cozmo.world.CameraImage, **kw):
      #  cv.destroyAllWindows()

        raw_img = np.array(image.raw_image)
                

        
                
        crop_img = raw_img[int(raw_img.shape[0] * 0.60): raw_img.shape[0], int(raw_img.shape[1] * 0.0): int(raw_img.shape[1] * 1)]
                
        gray_img = cv.cvtColor(crop_img, cv.COLOR_BGR2GRAY)
        
        blurred_gray_img = cv.GaussianBlur(gray_img, (5,5), 0)
        
        ret, thresh = cv.threshold(blurred_gray_img, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
        
 
        
        _, contours, hierarchy = cv.findContours(thresh.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
                c = max(contours, key = cv.contourArea)
                M = cv.moments(c)
                
                #cv.drawContours(crop_img, contours, -1, (0, 255, 0), 1)

                cv.imshow('frame', crop_img)
                cv.waitKey(0)
        
                
                cx = int(M['m10'] / M['m00'])
                
                print(cx)
                
                if cx >= 120:
                        print("Turn Left")
                
                if cx < 120 and cx > 50:
                        
                        print("On track!")
                if cx <=50:
                        print("Turn right")
                        
        else:
                print("No line")
      #  raw_rgb = np.array(raw_img)
        
       # r, g, b = cv.split(raw_rgb)
        
        #hsv_img = cv.cvtColor(np.array(raw_img), cv.COLOR_RGB2HSV)
        
       # h, s, v = cv.split(hsv_img)
        
        #mer_img = cv.merge((h, s, v))
        
        #hsv_img = mer_img
        
        #rgb_img2 = cv.cvtColor(hsv_img, cv.COLOR_HSV2RGB)
        
        #invGamma = 1.0 / ((self._tk_ga_scale.get()+1) / 50 )
        #new = np.zeros(256)
        #ori = np.zeros(256)
        #for i in range(256):
         #   new[i] = ((i / 255.0) ** invGamma) * 255
          #  ori[i] = i
        #try:
         #   incr_ch_lut = self.create_LUT_8UC1(ori, new)
          #  low = np.array([self._tk_hl_scale.get(),self._tk_sl_scale.get(),self._tk_vl_scale.get()], dtype="uint8")
        #except:
        #    sys.exit('Window Closed - Exiting')
        #high = np.array([self._tk_hh_scale.get(),self._tk_sh_scale.get(),self._tk_vh_scale.get()], dtype="uint8")
        
        #rgb_img = cv.LUT(raw_rgb, incr_ch_lut).astype(np.uint8)
        #rgb_img2 = cv.LUT(rgb_img2, incr_ch_lut).astype(np.uint8)
        
      #  rgb_img = cv.blur(rgb_img,(3,3))
        
       # thresh_img = cv.inRange(rgb_img2, low, high )
        
        #raw_rgb_conv=cv.cvtColor(np.array(raw_img), cv.COLOR_BGR2RGB)
        #rgb_img_conv=cv.cvtColor(np.array(rgb_img), cv.COLOR_BGR2RGB)
        #rgb_img2_conv=cv.cvtColor(np.array(rgb_img2), cv.COLOR_BGR2RGB)
        #cv.imwrite('raw_img.png', raw_rgb_conv)
        #cv.imwrite('rgb_img.png', rgb_img_conv)
        #cv.imwrite('rgb_img2.png', rgb_img2_conv)
    
        #pil_thresh = PIL.Image.fromarray(cv.cvtColor(thresh_img, cv.COLOR_GRAY2RGB))
        #rgb_img = PIL.Image.fromarray(rgb_img)
        #rgb_img2 = PIL.Image.fromarray(rgb_img2)
        

        #display_image_input = PIL.ImageTk.PhotoImage(image=pil_thresh)
        #display_image_output = PIL.ImageTk.PhotoImage(image=rgb_img2)
        #self._tk_label_input.imgtk = display_image_input
        #self._tk_label_input.configure(image=display_image_input)
        #self._tk_label_output.imgtk = display_image_output
        #self._tk_label_output.configure(image=display_image_output)
        #self._tk_root.update()
        
    
    def create_LUT_8UC1(self, x, y):
        spl = UnivariateSpline(x, y)
        return spl(range(256))

    async def set_up_cozmo(self, coz_conn):
        asyncio.set_event_loop(coz_conn._loop)
        self._robot = await coz_conn.wait_for_robot()
        self._robot.camera.image_stream_enabled = True
        self._robot.camera.color_image_enabled = True
        self._robot.add_event_handler(cozmo.world.EvtNewCameraImage, self.on_img)
        self._robot.set_head_angle(cozmo.util.Angle(degrees=0))

    async def run(self, coz_conn):
        await self.set_up_cozmo(coz_conn)
        
        #self._tk_root = tk.Tk()
        #self._tk_label_input = tk.Label(self._tk_root)
        #self._tk_label_output = tk.Label(self._tk_root)
        #self._tk_ga_scale = tk.Scale(self._tk_root, from_=0, to=255, orient=tk.HORIZONTAL, length=300, label='ga')
        #self._tk_hl_scale = tk.Scale(self._tk_root, from_=0, to=255, orient=tk.HORIZONTAL, length=300, label='hl')
        #self._tk_sl_scale = tk.Scale(self._tk_root, from_=0, to=255, orient=tk.HORIZONTAL, length=300, label='sl')
        #self._tk_vl_scale = tk.Scale(self._tk_root, from_=0, to=255, orient=tk.HORIZONTAL, length=300, label='vl')
        #self._tk_hh_scale = tk.Scale(self._tk_root, from_=0, to=255, orient=tk.HORIZONTAL, length=300, label='hh')
        #self._tk_sh_scale = tk.Scale(self._tk_root, from_=0, to=255, orient=tk.HORIZONTAL, length=300, label='sh')
        #self._tk_vh_scale = tk.Scale(self._tk_root, from_=0, to=255, orient=tk.HORIZONTAL, length=300, label='vh')
        #self._tk_label_input.pack()
        #self._tk_label_output.pack()
        #self._tk_ga_scale.pack()
        #self._tk_hl_scale.pack()
        #self._tk_sl_scale.pack()
        #self._tk_vl_scale.pack()
        #self._tk_hh_scale.pack()
        #self._tk_sh_scale.pack()
        #self._tk_vh_scale.pack()
        
        while True:
            await asyncio.sleep(0)


if __name__ == '__main__':
    Thresh()
