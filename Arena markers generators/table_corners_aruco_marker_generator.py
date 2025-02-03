import cv2
import cv2.aruco as aruco
import numpy as np
from PIL import Image
from reportlab.lib.pagesizes import A4
from reportlab.lib.utils import ImageReader
from reportlab.pdfgen import canvas

import os
import sys

if sys.platform == 'win32':
    # Get the absolute path of the directory containing the script
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Change the working directory to the script's directory
    os.chdir(script_dir)
    
corner_ids = [100,110]
idsetstr = "".join(str(id) for id in corner_ids)
marker_size=298
circle_diameter_cm = 12
border_thickness = 2
margin = 100
    
all_markers = np.ones(((margin + marker_size) * 2 + margin,margin*2+marker_size), np.uint8)*255
i = 0
for id in corner_ids:
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
    marker_image = aruco.generateImageMarker(aruco_dict, id, marker_size)
    all_markers[(margin+marker_size)*i+margin:(margin+marker_size)*i+margin+marker_size,margin:margin+marker_size] = marker_image
    i = i + 1

# Convert the single-channel marker image to three channels (grayscale to RGB)
all_markers = cv2.cvtColor(all_markers, cv2.COLOR_GRAY2RGB)

cv2.rectangle(all_markers, (int(margin/2),int(margin/2)), (int(margin/2+margin+marker_size),int(margin/2+margin+marker_size)), (0,0,0), 2)
cv2.rectangle(all_markers, (int(margin/2),int(2*margin + 2*marker_size + margin/2)), (int(margin/2+margin+marker_size),int(margin/2+margin+marker_size)), (0,0,0), 2)

cv2.imwrite(f"robot_label_{idsetstr}.jpg", all_markers)

output_pdf = canvas.Canvas(f"robot_label_{idsetstr}_{id}.pdf", pagesize=(all_markers.shape[1]+margin, all_markers.shape[0]+margin))
output_pdf.drawImage(ImageReader(f"robot_label_{idsetstr}.jpg"), int(margin/2), int(margin/2))
output_pdf.save()

