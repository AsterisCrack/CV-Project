import cv2
import glob
import copy
import math
import numpy as np
import imageio
import matplotlib.pyplot as plt

from pprint import pprint as pp
import os

def load_images(filenames):
    #DeprecationWarning: Starting with ImageIO v3 the behavior of this function will switch to that of iio.v3.imread. To keep the current behavior (and make this warning disappear) use `import imageio.v2 as imageio` or call `imageio.v2.imread` directly.
    #return [imageio.imread(filename) for filename in filenames]
    #To fix the warning:
    return [imageio.v3.imread(filename) for filename in filenames]

def get_chessboard_points(chessboard_shape, dx, dy):
    #IMPLEMENT this function. It shall return a vector of 3 components where the third is zero.
    #N = chessboard_shape[0] * chessboard_shape[1]
    points = [[i*dx, j*dy, 0] for i in range(chessboard_shape[0]) for j in range(chessboard_shape[1])]
    return np.array(points, dtype=np.float32)

def drawChessboardCorners(imgs_bw, corners):
    imgs_painted = []
    for i, cor in zip(imgs_bw, corners):
        if cor is not None and cor.any():
            cv2.drawChessboardCorners(i, (11, 10), cor, True)
            imgs_painted.append(i)
    return imgs_painted

#Load images
root = os.getcwd()
path_r = os.path.join(root, "calibration_images")
filenames_r = [i for i in glob.glob(path_r + '/*.jpg')]
imgs_r = load_images(filenames_r)
imgs_to_iter = load_images(filenames_r)

print(f"Numero de imagenes: {len(imgs_r)}\n")
#Get corners
corners_r = []

#print(f"Numero de imagenes pre-check: {len(imgs_r)}\n")

imgs_r_2 = []
for i, img in enumerate(imgs_to_iter):
    ret, c = cv2.findChessboardCorners(img, (11, 10)) #, None
    print(f"Imagen {i}: {ret}")
    print(c)
    # ret es un booleano que indica si se ha encontrado el tablero
    if ret:
        corners_r.append(c)
        imgs_r_2.append(img)
imgs_r = imgs_r_2

#print(f"Numero de imagenes post-check: {len(imgs)}")
#print(f"Numero de corners post-check: {len(corners)}\n")

# OPTIONAL => cornerSubPix is a destructive function. so we need to copy corners to avoid data loss
corners2 = copy.deepcopy(corners_r)

# termination criteria (https://docs.opencv.org/3.1.0/dc/dbb/tutorial_py_calibration.html)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)

# utilizaremos la función de openCV "cornerSubPix" para refinar el valor de las esquinas calculadas. OJO que esta función necesita 
# que las imágenes del patrón de calibración estén en escala de grises primero
# PASA Cada una de las imagenes la volvemos a blanco y negro
imgs_grey = [cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) for img in imgs_r] # list of all images in black and white

# For each image and corners we are going to use cornerSubPix
# cornersRefined = [cv2.cornerSubPix(i, cor[1], (11, 10), (-1, -1), criteria) if cor[0] else [] for i, cor in zip(imgs_grey, corners2)]
# pp(cornersRefined)

cornersRefined = []
print(len(imgs_grey))
print(len(corners2))
for i, cor in zip(imgs_grey, corners2):
    if cor is not None and cor.any():
        refined = cv2.cornerSubPix(i, cor, (11, 10), (-1, -1), criteria)
        cornersRefined.append(refined)

imgs2 = copy.deepcopy(imgs_r)
imgs_painted = drawChessboardCorners(imgs2[:], cornersRefined)

# display 3 images
plt.figure(figsize=(20, 20))
plt.subplot(131)
plt.imshow(imgs_painted[0])
plt.subplot(132)
plt.imshow(imgs_painted[1])
plt.subplot(133)
plt.imshow(imgs_painted[2])
plt.show()


cb_points = get_chessboard_points((11, 10), 16.5, 16.5)
#print(cb_points)

valid_corners = [cor for cor in cornersRefined if cor is not None and cor.any()]
num_valid_images = len(valid_corners)

# Matrix with the coordinates of the corners
real_points = get_chessboard_points((11, 10), 16.5, 16.5)

# We are going to convert our coordinates list in the reference system to numpy array
object_points = np.asarray([real_points for i in range(num_valid_images)], dtype=np.float32)

# Convert the corners list to array
image_points = np.asarray(valid_corners, dtype=np.float32)

# ASIGNMENT: Calibrate the left camera
rms_r, intrinsics_r, dist_coeffs_r, rvecs_r, tvecs_r = cv2.calibrateCamera(object_points, image_points, imgs_grey[0].shape[::-1], None, None)
# Calculate extrinsecs matrix using Rodigues on each rotation vector addid its translation vector
extrinsics_r = list(map(lambda rvec, tvec: np.hstack((cv2.Rodrigues(rvec)[0], tvec)), rvecs_r, tvecs_r))
# Save the calibration file
np.savez('calib_right', intrinsic=intrinsics_r, extrinsic=extrinsics_r)

# Lets print some outputs
print("Corners standard intrinsics:\n",intrinsics_r)
print("Corners standard dist_coefs:\n", dist_coeffs_r)
print("root mean sqaure reprojection error:\n", rms_r)


# Calculate the extrinsecs with Rodrigues given the translation and rotation matrixes
extrinsics_r = list(map(lambda rvec_r, tvec_r: np.hstack((cv2.Rodrigues(rvec_r)[0], tvec_r)), rvecs_r, tvecs_r))
print("Extrinsc:\n",extrinsics_r)


