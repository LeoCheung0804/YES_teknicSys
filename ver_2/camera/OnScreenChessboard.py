import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

def draw_chessboard(rows, cols, block_size_mm, monitor_width, monitor_height, screen_size_inches):
    # Convert screen size from inches to millimeters
    screen_size_mm = screen_size_inches * 25.4

    # Calculate the aspect ratio of the monitor
    aspect_ratio = monitor_width / monitor_height

    # Calculate the physical width and height in millimeters
    monitor_physical_width = (screen_size_mm ** 2 / (1 + aspect_ratio ** 2)) ** 0.5 * aspect_ratio
    monitor_physical_height = monitor_physical_width / aspect_ratio

    # Calculate the pixel size per millimeter for both dimensions
    pixel_width_per_mm = monitor_width / monitor_physical_width
    pixel_height_per_mm = monitor_height / monitor_physical_height

    # Calculate the size of each square in pixels
    square_width_pixels = int(block_size_mm * pixel_width_per_mm)
    square_height_pixels = int(block_size_mm * pixel_height_per_mm)

    # Calculate the total width and height of the chessboard in pixels
    chessboard_width_pixels = cols * square_width_pixels
    chessboard_height_pixels = rows * square_height_pixels

    # Create a blank image with a white background matching the chessboard size
    chessboard = np.ones((chessboard_height_pixels, chessboard_width_pixels, 3), dtype=np.uint8) * 255

    # Draw the chessboard
    for row in range(rows):
        for col in range(cols):
            # Determine the color of the square
            if (row + col) % 2 == 0:
                color = (255, 255, 255)  # White
            else:
                color = (0, 0, 0)  # Black

            # Calculate the top-left corner of the square
            top_left_x = col * square_width_pixels
            top_left_y = row * square_height_pixels

            # Draw the square
            cv2.rectangle(chessboard, (top_left_x, top_left_y), 
                          (top_left_x + square_width_pixels, top_left_y + square_height_pixels), 
                          color, -1)

    return chessboard

def save_to_pdf(chessboard, block_size_mm, rows, cols, filename="chessboard.pdf"):
    # Convert from BGR (OpenCV) to RGB (Matplotlib)
    rgb_chessboard = cv2.cvtColor(chessboard, cv2.COLOR_BGR2RGB)
    
    # Create a new figure with size in inches (converting from mm to inches)
    fig_width = cols * block_size_mm / 25.4  # convert mm to inches
    fig_height = rows * block_size_mm / 25.4
    
    fig = plt.figure(figsize=(fig_width, fig_height))
    
    # Remove margins and axes
    ax = plt.Axes(fig, [0., 0., 1., 1.])
    ax.set_axis_off()
    fig.add_axes(ax)
    
    # Display the image
    ax.imshow(rgb_chessboard)
    
    # Save to PDF with correct dimensions
    with PdfPages(filename) as pdf:
        pdf.savefig(fig, bbox_inches='tight', pad_inches=0)
    
    plt.close()

def main():
    # Input the number of rows, columns, block size, monitor resolution, and screen size
    rows = 6
    cols = 7
    block_size_mm = 30
    monitor_width = 1920  # Monitor width in pixels
    monitor_height = 1080  # Monitor height in pixels
    screen_size_inches = 24

    # Draw the chessboard
    chessboard = draw_chessboard(rows, cols, block_size_mm, monitor_width, monitor_height, screen_size_inches)

    # Save to PDF
    save_to_pdf(chessboard, block_size_mm, rows, cols)
    
    # Display the chessboard
    cv2.imshow('Chessboard Calibrator', chessboard)

    # Wait for a key press and close the window
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
