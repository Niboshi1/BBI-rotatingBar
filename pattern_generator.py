import mmap
from io import StringIO
from struct import unpack
import socket
import numpy as np
import time
import sys
import cv2

# Create rotating bar
def Convert(img8):
    s=img8.shape
    a=img8.reshape(s[0]*s[1]//8, 8)
    a2 = np.ones(a.shape)
    for i in range(7):
        a2[:, i] = 1 << (7 - i)
    a = a*a2
    return a.sum(axis=1)

def draw_bar(angle, w, h):
    img = np.zeros((h, w))
    radius = int(w*0.3)

    x = int(w/2 + np.cos(angle)*(radius))
    y = int(h/2 + np.sin(angle)*(radius))
    
    line_thickness = 10
    cv2.line(img, (int(w/2), int(h/2)), (x, y), 255, thickness=line_thickness)
    cv2.circle(img, (int(w/2), int(h/2)), radius, 255, 2)
    #cv2.rectangle(img, (0, 0), (w, h), 255, 3)
    cv2.imwrite("img/frame_{}.png".format(angle), img)
    
    img = ((img > 127) * np.ones(img.shape)).astype(np.uint8)
    
    n = w % 8
    if n != 0:
        pad = np.zeros((img.shape[0], 8 - n), dtype=np.uint8)
        img = np.hstack((img, pad))
    
    out = Convert(img).astype(np.uint8)
    
    return out

# Note the size here , It must not be greater than C++ The size of shared memory opened in , Otherwise, the mapping fails
SHARE_MEMORY_FILE_SIZE_BYTES = 256

# define TCP server address
host = "localhost"
port = 2222

img_w = 745
img_h = 552
angle = 0
w = np.uint32(img_w)
h = np.uint32(img_h)
func = np.uint32(1)

# connect to server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
remote_ip = socket.gethostbyname(host)
s.connect((remote_ip, port))

while True:
    mmap_file = mmap.mmap(-1, SHARE_MEMORY_FILE_SIZE_BYTES, "my_mapping", access=mmap.ACCESS_READ)
    #buffer_size = unpack("i", mmap_file.read(4))
    #print(f"Buffer size: {buffer_size}")
    msg = mmap_file.read().decode()
    print(msg)
    msg_int = ""
    for i in [0, 2, 4, 6, 8, 10, 12, 14, 16]:
        msg_int += msg[i]
    
    # Send image to server
    # Below, the variable can be changed to either "1" or "2", which affects how the image is interpreted by PolyScan2.
    # If "1", the uploaded image will fit within the EWA of the Polygon
    # (e.g. entire image will be seen in EWA)
    # If "2", the uploaded image will be truncated, so only the portion of the image within the EWA will be projected.
    # (e.g. image fits within camera window, but only the portion of the image seen in EWA will be projected)

    # send binary pattern to server ( 1bit/pixel )
    
    s.send(func)
    s.send(w)
    s.send(h)
    s.send(draw_bar(float(msg_int), img_w, img_h))
    
    time.sleep(1/10)

# close TCP connection
s.close()