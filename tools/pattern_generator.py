import numpy as np
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

def draw_bar(angle, w, h, draw=True):
    '''
    Returns a binary image with an angled bar

            Parameters:
                    angle (float): The angle to be drawn on the output image
                    w (int): The width of the output image
                    h (int): The hight of the output image

            Returns:
                    out (numpy.array): np.array of the output image with an angled bar drawn
    '''
    img = np.zeros((h, w))
    radius = int(w*0.3)
    radius_in = int(w*0.1)

    x = int(w/2 + np.cos(angle)*(radius))
    y = int(h/2 - np.sin(angle)*(radius))

    x_in = int(w/2 + np.cos(angle)*(radius_in))
    y_in = int(h/2 - np.sin(angle)*(radius_in))
    
    line_thickness = 10

    if draw == True:
        cv2.line(img, (x_in, y_in), (x, y), 255, thickness=line_thickness)
    #cv2.circle(img, (int(w/2), int(h/2)), radius, 255, 2)
    #cv2.rectangle(img, (0, 0), (w, h), 255, 3)
    #cv2.imwrite("img/frame_{}.png".format(angle), img)
    
    img = ((img > 127) * np.ones(img.shape)).astype(np.uint8)
    
    n = w % 8
    if n != 0:
        pad = np.zeros((img.shape[0], 8 - n), dtype=np.uint8)
        img = np.hstack((img, pad))
    
    out = Convert(img).astype(np.uint8)
    
    return out