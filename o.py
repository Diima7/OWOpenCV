import cv2
from math import pow, sqrt
from mss import mss
import numpy as np
from win32 import win32api, win32gui
import win32con
from random import randint
import keyboard, mouse, pyautogui, time, os, sys
from ctypes import cdll, byref, create_string_buffer
from ctypes.util import find_library
SQUARE_SIZE = 300
AIMHEAD = False
SPEED = 0.4  #Smoothness/Speed
CONFIDENCE = 0.3

TARGET_SIZE = 120
MAX_TARGET_DISTANCE = sqrt(2 * pow(TARGET_SIZE, 2))
CONFIDENCE = 1 - CONFIDENCE

sct = mss()

dimensions = sct.monitors[1]

dimensions['left'] = int((dimensions['width'] / 2) - (SQUARE_SIZE / 2))
dimensions['top'] = int((dimensions['height'] / 2) - (SQUARE_SIZE / 2))
dimensions['width'] = SQUARE_SIZE
dimensions['height'] = SQUARE_SIZE


class assistent:
    def __init__(self, AIMHEAD, SPEED, CONFIDENCE, SHOW_FRAME, SMOOTH):
        # Dunkeles Magenta
        H_LOW = 139
        S_LOW = 96
        V_LOW = 129

        # Helles Magenta
        H_HIGH = 169
        S_HIGH = 225
        V_HIGH = 225
        
        self.magenta_dark = np.array([H_LOW, S_LOW, V_LOW])
        self.magenta_light = np.array([H_HIGH, S_HIGH, V_HIGH])
        
        self.smooth = SMOOTH
        self.aimhead = AIMHEAD
        self.speed = SPEED
        self.confidence = CONFIDENCE
        self.show_frame = SHOW_FRAME
    def mouse_move(self, x, y):
        if self.smooth:
            mouse.move(x, y, absolute=False, duration=0.08)
        else:
            win32api.mouse_event(win32con.MOUSEEVENTF_MOVE, x, y, 0, 0)
        #mouse.move(x, y, absolute=False, duration=0.2)
    def is_activated(self):
        return win32api.GetAsyncKeyState(0x14) != 0
    
    def process(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        magenta_color_range = cv2.inRange(hsv, self.magenta_dark, self.magenta_light)

        res = cv2.bitwise_or(frame, frame, mask=magenta_color_range)
        res = cv2.cvtColor(res, cv2.COLOR_RGB2GRAY)

        # magenta Farben filtern
        thresh = cv2.adaptiveThreshold(res, 1, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                    cv2.THRESH_BINARY, 15, 1)

        # magenta umrandung finden
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Erstelle ein convex hull polygon um jedes gefundene Objekt
        contours = map(lambda ct: cv2.convexHull(ct, False), contours)

        # Finde das größte Objekt
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        if __debug__:
            if self.show_frame:
                cv2.drawContours(frame, contours, -1, (255, 255, 0), 1)

        # Filtere einzelne Fragmente
        return list(filter(self.contour_filter, contours))
    
    def contour_distance(self, ct):
        global SQUARE_SIZE
        moment = cv2.moments(ct)
        if moment["m00"] == 0:
            return -1, None

        cx = int(moment["m10"] / moment["m00"])
        cy = int(moment["m01"] / moment["m00"])

        mid = SQUARE_SIZE / 2
        x = (mid - cx) if cx < mid else cx - mid
        y = (mid - cy) if cy < mid else cy - mid
        return [x, y]
    
    def contour_filter(self, ct):
        global TARGET_SIZE
        x, y = self.contour_distance(ct)
        
        # Überprüfe ob das Ziel/Objekt im eingestellten Bereich des Benutzers liegt.
        if x == -1 or x > TARGET_SIZE or y > TARGET_SIZE:
            return False

        # Lösche zu kleine Objekte
        if cv2.contourArea(ct) < 1000:
            return False

        # Berechne die Höhe und Breite der gefundenen Kontur
        extreme_left = tuple(ct[ct[:, :, 0].argmin()][0])
        extreme_right = tuple(ct[ct[:, :, 0].argmax()][0])
        extreme_top = tuple(ct[ct[:, :, 1].argmin()][0])
        extreme_bottom = tuple(ct[ct[:, :, 1].argmax()][0])
        width = extreme_right[0] - extreme_left[0]
        height = extreme_bottom[1] - extreme_top[1]

        if width == 0 or height == 0:
            return False

        # Filtere zu breite Objekte
        if abs(width / height) > 2.5:
            return False

        return True
    
    def locate_target(self, target, frame):
        global SQUARE_SIZE
        global MAX_TARGET_DISTANCE
        # Berechne die mitte der Kontur
        moment = cv2.moments(target)
        if moment["m00"] == 0:
            return

        xpn, ypn = randint(0,1), randint(0,1)
        randx, randy = randint(1,30), randint(1,30)
        
        cx = int((moment["m10"] / moment["m00"]))
        cy = int((moment["m01"] / moment["m00"]) * 0.9)
        
        if xpn == 0:
            cx -= int(randx * self.confidence)
        else:
            cx += int(randx * self.confidence)
            
        if ypn == 0:
            cy -= int(randy * self.confidence)
        else:
            cy += int(randy * self.confidence)
        
        mid = SQUARE_SIZE / 2
        x = -(mid - cx) if cx < mid else cx - mid
        #if self.aimhead:
        #    mid = (SQUARE_SIZE / 2) * 1.1
        y = -(mid - cy) if cy < mid else cy - mid
        
        target_size = cv2.contourArea(target)
        distance = sqrt(pow(x, 2) + pow(y, 2))
	# Wichtige Werte, müssen bei anderen Spielen an die Maus sensitivität,
	# der größe des Objektes und der Entfernung angepasst werden.
        slope = ((1.0 / 3.0) - 1.0) / (MAX_TARGET_DISTANCE / target_size)
        multiplier = ((MAX_TARGET_DISTANCE - distance) / target_size) * slope + 1
        self.MarkTarget(cx, cy)
        if self.is_activated():
            x = int((x * multiplier) * self.speed)
            y = int((y * multiplier) * self.speed)
            self.mouse_move(x, y)

        if __debug__:
            if self.show_frame:
                # Macht die Kontur des ausgewählten Objektes im Vorschau Fenster Grün
                cv2.drawContours(frame, [target], -1, (0, 255, 0), 2)
                # Platziert einen weißen kreis in die mitte des Objektes
                cv2.circle(frame, (cx, cy), 7, (255, 255, 255), -1)
    
    def MarkTarget(self,x,y):
        dc = win32gui.GetDC(0)
        red = win32api.RGB(255, 0, 0)
        nx = x - int(SQUARE_SIZE / 12)
        ny = y - int(SQUARE_SIZE / 12)
        print(x,y)
        for i in range(int(SQUARE_SIZE / 6)):
            win32gui.SetPixel(dc, int((dimensions['left'] + nx) + i), int(dimensions['top'] + y), red) #links
            win32gui.SetPixel(dc, int((dimensions['left'] + x)), int((dimensions['top'] + ny) + i), red) #links
            #win32gui.Set
            

        #time.sleep(1)
            
    def drawSquare(self):
        dc = win32gui.GetDC(0)
        red = win32api.RGB(255, 0, 0)
        for i in range(SQUARE_SIZE):
            win32gui.SetPixel(dc, dimensions['left'], dimensions['top'] + i, red) #links
            win32gui.SetPixel(dc, dimensions['left'] + i, dimensions['top'], red) #oben
            
            win32gui.SetPixel(dc, dimensions['left'] + SQUARE_SIZE, dimensions['top'] + i, red) #links
            win32gui.SetPixel(dc, dimensions['left'] + i, dimensions['top'] + SQUARE_SIZE, red) #oben
        time.sleep(1)
         
    def start(self):
        global dimensions
        global sct
        #print('Starting..')
        self.drawSquare()
        while True:
            #fps_counter = time.time()
            frame = np.asarray(sct.grab(dimensions))
            contours = self.process(frame)

            # Versuche das Ziel welches am meisten mit den eingestellten Konturen übereinstimmt anzuvisieren
            if len(contours) > 1:
                # contour[1] == Der größte und dem Zielvisier am nächsten liegende Charakter
                self.locate_target(contours[1], frame)
            if __debug__:
                if self.show_frame:
                    cv2.drawContours(frame, contours, -1, (0, 255, 0), 1)
                    cv2.imshow('res', frame)
            #    pass
            # "q" drücken um das Programm zu beenden
            if cv2.waitKey(25) & 0xFF == ord("q"):
                break
            if keyboard.is_pressed('q'):
                break
  
  
  
  
  
  
def set_proc_name(newname):
    libc = cdll.LoadLibrary('libc.so.6')
    buff = create_strin
    g_buffer(len(newname)+1)
    buff.value = newname
    libc.prctl(15, byref(buff), 0, 0, 0)

def get_proc_name():
    libc = cdll.LoadLibrary('libc.so.6')
    buff = create_string_buffer(128)
    # 16 == PR_GET_NAME from <linux/prctl.h>
    libc.prctl(16, byref(buff), 0, 0, 0)
    return buff.value



              
b = assistent(AIMHEAD, SPEED, CONFIDENCE, False, True)
b.start()

sct.close()
cv2.destroyAllWindows()
