import cv2
import numpy as np

# Read the CSV file
brick_data = np.loadtxt(r'bin\NOHOLE14.csv', delimiter=',')

# Create a blank image
image_height, image_width = 600, 800
image = np.zeros((image_height, image_width, 3), dtype=np.uint8)

# Define a function to draw bricks
def draw_bricks(image, brick_data):
    for brick in brick_data:
        x, y, _, rotation = brick
        x = int(x * 50)  # Scale the x coordinate
        y = int(y * 50)  # Scale the y coordinate
        width, height = 5, 2  # Brick dimensions

        # Create a rectangle and rotate it
        rect = ((x + width // 2, y + height // 2), (width, height), rotation)
        print(rect)
        box = cv2.boxPoints(rect)
        box = np.int8(box)
        print(box)
        cv2.drawContours(image, [box], 0, (0, 255, 0), -1)

# Draw the bricks on the image
draw_bricks(image, brick_data)

# Display the image
cv2.imshow('Bricks', image)
cv2.waitKey(0)
cv2.destroyAllWindows()