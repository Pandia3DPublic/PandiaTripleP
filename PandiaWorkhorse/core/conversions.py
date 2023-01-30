import cv2
from base64 import b64encode

#quality: [0,100] the higher the better the quality however less compression thus higher filesize
#targetWidth: for resizing to mainly smaller dimensions for web bandwidth efficiency
def numpyToUTF8JPG(img, quality=40, targetWidth=-1):
    if targetWidth > 10:
        height, width, ch = img.shape
        if width > targetWidth:
            ar = width / height
            targetHeight = targetWidth / ar
            img = cv2.resize(img, (int(targetWidth), int(targetHeight)), interpolation=cv2.INTER_AREA)
    success, jpg_img = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if success:
        return b64encode(jpg_img.tobytes()).decode('utf-8')
    else:
        print("Encoding failed")
        return False