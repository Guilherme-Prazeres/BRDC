#!/usr/bin/env python3

from __future__ import print_function
#from six.moves import input


import cv2
import numpy as np
from pdf2image import convert_from_path
import matplotlib.pyplot as plt
from PIL import Image
from pypdf import PdfReader
import time

# DEFAULT SCALE IN MILIMITERS
OFFSET = 3 #mm
pdf_path = 'path01.pdf'

def get_pdf_size(pdf_path):
    with open(pdf_path, 'rb') as file:
        reader = PdfReader(file)
        box = reader.pages[0].mediabox
        width = float(box.width)
        height = float(box.height)
        
        # Assuming the PDF uses points as the unit of measurement
        # Convert points to millimeters (1 point = 0.3528 mm)
        width_mm = width * 0.3528
        height_mm = height * 0.3528

    return width_mm, height_mm

def pdf_curves_to_dots(pdf_path, min_distance=1):
    # Convert PDF to images
    images = convert_from_path(pdf_path, grayscale=True)

    all_dots = []
    for image in images:
        # Convert image to numpy array
        img_array = np.array(image)

        # Threshold the image to extract black curves
        _, threshold = cv2.threshold(img_array, 0, 255, cv2.THRESH_BINARY_INV)

        # Find contours of black curves
        contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Convert contours to dots
        for contour in contours:
            for point in contour:
                x, y = point[0]
                dot = (x, y)
                if not is_close(dot, all_dots, min_distance):
                    all_dots.append(dot)

    return all_dots

def is_close(dot, existing_dots, min_distance):
    for existing_dot in existing_dots:
        distance = np.linalg.norm(np.array(dot) - np.array(existing_dot))
        if distance <= min_distance:
            return True
    return False


# Convert PDF curves to dots

width_mm, height_mm = get_pdf_size(pdf_path)
print(f"Width: {round(width_mm,1)} mm")
print(f"Height: {round(height_mm,1)} mm")

dots = pdf_curves_to_dots(pdf_path, min_distance=5)

print("Number of dots: " + str(len(dots)))

# Print the dots
# print("Number of dots: " + str(len(dots)))
# for dot in dots:
#     print(dot)


# Load the PDF page as an image
images = convert_from_path(pdf_path)
image = images[0].convert("L")  # Convert to grayscale


# Scale the dots according to the document size
scale_factor_x = width_mm / image.width
scale_factor_y = height_mm / image.height
scaled_dots = [(dot[0] * scale_factor_x, width_mm - dot[1] * scale_factor_y) for dot in dots]



# Convert the image to a numpy array
img_array = np.array(image)

# Display the image with dots
plt.imshow(img_array, cmap='gray', extent=(0, width_mm, 0, height_mm))
plt.plot(*zip(*scaled_dots), '.', color='red', markersize=1)
plt.show()

# Transform dots into a 3D array with constant offset
dots_3d = [(x, y, OFFSET) for x, y in scaled_dots]

# Print the transformed dots
# for dot in dots_3d:
#     print(dot)


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Extract x, y, and z coordinates from the dots
x_coords, y_coords, z_coords = zip(*[(dot[0], dot[1], dot[2]) for dot in dots_3d])

# Plot the dots in 3D space
ax.scatter(x_coords, y_coords, z_coords, c='r', marker='o')

# Set labels and title for the plot
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Dots in 3D')

# Show the 3D plot
plt.show()

np.savetxt('pathDots.txt', dots_3d)