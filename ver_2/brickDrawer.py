import cv2
import numpy as np
import pandas as pd

def draw_brick(image, x, y, z, rotation, index, brick_color=(0, 0, 255)):
    # Define brick dimensions (you can adjust these)
    brick_width = 30
    brick_height = 15
    
    # Create rectangle points
    rect = np.array([
        [-brick_width/2, -brick_height/2],
        [brick_width/2, -brick_height/2],
        [brick_width/2, brick_height/2],
        [-brick_width/2, brick_height/2]
    ], dtype=np.float32)
    
    # Apply rotation
    angle = np.radians(rotation)
    rotation_matrix = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle), np.cos(angle)]
    ])
    rect = np.dot(rect, rotation_matrix.T)
    
    # Translate to position (x,y)
    rect += np.array([x, y])
    
    # Convert to integer points
    rect = rect.astype(np.int32)
    
    # Draw the brick
    cv2.fillPoly(image, [rect], brick_color)
    cv2.polylines(image, [rect], True, (0, 0, 0), 1)  # Black border
    
    # Add index number
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.4
    text_color = (0, 0, 0)  # Black text
    text_thickness = 1
    text = str(index)
    
    # Get text size to center it in the brick
    (text_width, text_height), _ = cv2.getTextSize(text, font, font_scale, text_thickness)
    text_x = int(x - text_width/2)
    text_y = int(y + text_height/2)
    
    cv2.putText(image, text, (text_x, text_y), font, font_scale, text_color, text_thickness)

def main():
    # Read CSV file without headers
    filename = r'bin/NOHOLE40.csv'
    try:
        df = pd.read_csv(filename, header=None, names=['x', 'y', 'z', 'rotation'])
    except FileNotFoundError:
        print("Error: " + filename + " file not found!")
        return
    
    # Create a white canvas
    canvas_width = 800
    canvas_height = 600
    image = np.ones((canvas_height, canvas_width, 3), dtype=np.uint8) * 255
    
    # Find data ranges for better scaling
    x_min, x_max = df['x'].min(), df['x'].max()
    y_min, y_max = df['y'].min(), df['y'].max()
    
    # Draw each brick
    for index, row in df.iterrows():
        # Scale coordinates to fit canvas with padding
        padding = 50  # pixels from edge
        x = int(((row['y'] - y_min) / (y_max - y_min)) * (canvas_width - 2*padding) + padding)
        y = int(((row['x'] - x_min) / (x_max - x_min)) * (canvas_height - 2*padding) + padding)
        
        # Use z-value to determine color intensity (closer bricks are darker)
        color_intensity = max(50, min(255, 255 + int(row['z'] * 20)))  # Adjusted for negative z values
        brick_color = (0, 0, color_intensity)
        
        draw_brick(image, x, y, row['z'], -row['rotation'] + 90, index, brick_color)
    
    # Display the result
    cv2.imshow('Brick Layout', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
