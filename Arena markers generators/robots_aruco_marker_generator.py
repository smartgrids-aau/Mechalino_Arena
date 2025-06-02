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
    
def put_image_in_circle(image, circle_diameter_cm, border_thickness):
    # Calculate circle diameter in pixels based on dpi=300 (1 inch = 2.54 cm)
    circle_diameter = int(circle_diameter_cm * 71.59 / 2.54)

    if max(image.shape[:2]) > circle_diameter:
        raise ValueError("Image dimensions are larger than the specified circle diameter.")

    # Calculate the center of the circle
    center_x = circle_diameter // 2
    center_y = circle_diameter // 2

    # Create a white background image with the specified circle diameter
    circle_image = np.ones((circle_diameter, circle_diameter, 3), dtype=np.uint8) * 255

    # Calculate the radius of the circle
    radius = (circle_diameter - 2 * border_thickness) // 2

    # Draw the circle on the white background
    cv2.circle(circle_image, (center_x, center_y), radius, (0, 0, 0), border_thickness)

    # Calculate the region where the original image will be placed inside the circle
    start_x = center_x - radius + border_thickness
    end_x = center_x + radius - border_thickness
    start_y = center_y - radius + border_thickness
    end_y = center_y + radius - border_thickness

    # Get the dimensions of the input image
    input_height, input_width = image.shape[:2]

    # Calculate the position to paste the image inside the circle
    paste_x = (end_x - start_x - input_width) // 2 + start_x
    paste_y = (end_y - start_y - input_height) // 2 + start_y

    # Paste the input image inside the circle on the white background
    circle_image[paste_y:paste_y + input_height, paste_x:paste_x + input_width] = image

    return circle_image

id = 15

marker_size= 189
circle_diameter_cm = 12
border_thickness = 2
margin = 80

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)

marker_image = aruco.generateImageMarker(aruco_dict, id, marker_size)

# Convert the single-channel marker image to three channels (grayscale to RGB)
marker_image_rgb = cv2.cvtColor(marker_image, cv2.COLOR_GRAY2RGB)


output_image = put_image_in_circle(marker_image_rgb, circle_diameter_cm, border_thickness)
hs = output_image.shape[0]//2
k = 12
cv2.rectangle(output_image,(0,hs-k),(3*k,hs+k),[180,180,180])
cv2.rectangle(output_image,(hs*2-3*k,hs-k),(hs*2,hs+k),[180,180,180])
cv2.rectangle(output_image,(hs-k,0),(hs+k,3*k),[180,180,180])
cv2.rectangle(output_image,(hs-k,hs*2-3*k),(hs+k,hs*2),[180,180,180])
cv2.imwrite(f"robot_label_{id}.jpg", output_image)

output_pdf = canvas.Canvas(f"robot_label_{id}.pdf", pagesize=(output_image.shape[1]+margin, output_image.shape[0]+margin))
output_pdf.drawImage(ImageReader(f"robot_label_{id}.jpg"), int(margin/2), int(margin/2))
output_pdf.save()
