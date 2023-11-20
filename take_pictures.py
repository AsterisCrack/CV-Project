import cv2
from picamera2 import Picamera2

picam = Picamera2()
picam.preview_configuration.main.size=(1280, 720)
picam.preview_configuration.main.format="RGB888"
picam.preview_configuration.align()
picam.configure("preview")
picam.start()

def take_picture(save_path):
    frame = picam.capture_array()
    cv2.imwrite(save_path, frame)
    cv2.destroyAllWindows()

def take_multiple_pictures(save_path):
    i=0
    while True:
        print("Press any key to take another picture, or q to quit")
        inp = input(">> ")
        if inp == "q":
            break

        take_picture(save_path + str(i) + ".jpg")
        i+=1
        print("Picture taken")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    take_multiple_pictures("calibration_images/image_")
    