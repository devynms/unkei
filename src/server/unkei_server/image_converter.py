import cv2
import math
import pdb

def img_converter(filename):
    vid = cv2.VideoCapture(filename)
    print 'creating images'
    ret, frame = vid.read()
    i = 0
    dec = 10
    while ret:
        zeros = '0'
        if(i >= dec):
            dec *= 10 
        zeros = zeros*int(10 - math.log(dec, 10))
        cv2.imwrite('images/pic{0}{1}.png'.format(zeros,i), frame)    
        ret, frame = vid.read()
        i += 1
