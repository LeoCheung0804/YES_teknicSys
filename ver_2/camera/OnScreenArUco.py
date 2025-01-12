import cv2
import cv2.aruco as aruco
import math
from matplotlib import pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

def generate_aruco_marker(marker_id, marker_size_mm, screen_resolution, screen_diagonal_inches, aruco_dict_type):
    
    screen_size_mm = screen_diagonal_inches * 25.4

    # Calculate the aspect ratio of the monitor
    aspect_ratio = screen_resolution[0] / screen_resolution[1]

    # Calculate the physical width and height in millimeters
    monitor_physical_width = (screen_size_mm ** 2 / (1 + aspect_ratio ** 2)) ** 0.5 * aspect_ratio
    monitor_physical_height = monitor_physical_width / aspect_ratio

    # Calculate the pixel size per millimeter for both dimensions
    pixel_width_per_mm = screen_resolution[0] / monitor_physical_width
    pixel_height_per_mm = screen_resolution[1] / monitor_physical_height

    # Calculate marker size in pixels
    marker_size_pixels = int(marker_size_mm * pixel_width_per_mm)

    # Define the dictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco_dict_type)

    # Generate the marker
    marker_image = aruco.generateImageMarker(aruco_dict, marker_id, marker_size_pixels)

    # Convert mm to inches for PDF sizing
    marker_size_inches = marker_size_mm / 25.4
    
    # Save to PDF with accurate physical size
    with PdfPages('aruco_marker.pdf') as pdf:
        # A4 size in inches (8.27 x 11.69)
        fig = plt.figure(figsize=(8.27, 11.69))
        
        # Calculate the figure size in relative units to maintain physical size
        fig_width_inches = fig.get_size_inches()[0]
        marker_relative_size = marker_size_inches / fig_width_inches
        
        # Create axes with specific size and centered position
        ax = plt.axes([
            (1 - marker_relative_size) / 2,  # x position
            (1 - marker_relative_size) / 2,  # y position
            marker_relative_size,            # width
            marker_relative_size             # height
        ])
        
        ax.imshow(marker_image, cmap='gray')
        ax.axis('off')
        pdf.savefig()
        plt.close()

    # Create a window to display the marker
    cv2.namedWindow('ArUco Marker', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('ArUco Marker', marker_size_pixels, marker_size_pixels)

    # Display the marker
    cv2.imshow('ArUco Marker', marker_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def generate_aruco_board(board_width, board_height, marker_size_mm, marker_separation_mm, screen_resolution, screen_diagonal_inches, aruco_dict_type):
    screen_size_mm = screen_diagonal_inches * 25.4

    # Calculate the aspect ratio of the monitor
    aspect_ratio = screen_resolution[0] / screen_resolution[1]

    # Calculate the physical width and height in millimeters
    monitor_physical_width = (screen_size_mm ** 2 / (1 + aspect_ratio ** 2)) ** 0.5 * aspect_ratio

    # Calculate the pixel size per millimeter for both dimensions
    pixel_width_per_mm = screen_resolution[0] / monitor_physical_width

    # Calculate marker size and separation in pixels
    marker_size_pixels = int(marker_size_mm * pixel_width_per_mm)
    marker_separation_pixels = int(marker_separation_mm * pixel_width_per_mm)

    # Define the dictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco_dict_type)

    # Create the board
    board = cv2.aruco.GridBoard((board_width, board_height), 
                               marker_size_mm,
                               marker_separation_mm,
                               aruco_dict)

    # Calculate the board image size
    board_width_pixels = board_width * marker_size_pixels + (board_width - 1) * marker_separation_pixels
    board_height_pixels = board_height * marker_size_pixels + (board_height - 1) * marker_separation_pixels
    
    # Draw the board (reducing margin and border size)
    margin_size = 5
    boarder_bits = 1
    board_image = cv2.aruco.drawPlanarBoard(board, 
                                          (board_width_pixels, board_height_pixels),
                                          margin_size,
                                          boarder_bits)

    # Convert mm to inches for PDF sizing
    board_width_mm = board_width * marker_size_mm + (board_width - 1) * marker_separation_mm
    board_height_mm = board_height * marker_size_mm + (board_height - 1) * marker_separation_mm
    board_width_inches = board_width_mm / 25.4
    board_height_inches = board_height_mm / 25.4
    
    # Save to PDF with accurate physical size
    with PdfPages('aruco_board.pdf') as pdf:
        # A4 size in inches (8.27 x 11.69)
        fig = plt.figure(figsize=(8.27, 11.69))
        
        # Calculate the scaling factor to fit the board while maintaining aspect ratio
        page_width = 8.27
        page_height = 11.69
        width_ratio = board_width_inches / page_width
        height_ratio = board_height_inches / page_height
        scale = min(1.0, 1/max(width_ratio, height_ratio))
        
        # Calculate relative size and position to center the board
        board_relative_width = (board_width_inches * scale) / page_width
        board_relative_height = (board_height_inches * scale) / page_height
        
        # Create axes with specific size and centered position
        ax = plt.axes([
            (1 - board_relative_width) / 2,   # x position
            (1 - board_relative_height) / 2,  # y position
            board_relative_width,             # width
            board_relative_height             # height
        ])
        
        ax.imshow(board_image, cmap='gray')
        ax.axis('off')
        pdf.savefig()
        plt.close()

    # Create a window to display the board
    cv2.namedWindow('ArUco Board', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('ArUco Board', board_image.shape[1], board_image.shape[0])

    # Display the board
    cv2.imshow('ArUco Board', board_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Example usage
screen_resolution = (1920, 1080)  # Screen resolution in pixels (width, height)
screen_diagonal_inches = 24  # Diagonal screen size in inches
marker_size_mm = 50  # Desired marker size in millimeters
marker_separation_mm = 10  # Separation between markers in millimeters
aruco_dict_type = aruco.DICT_4X4_50  # ArUco dictionary type

# generate_aruco_marker(marker_id=42, marker_size_mm=marker_size_mm, 
#                       screen_resolution=screen_resolution, screen_diagonal_inches=screen_diagonal_inches,
#                       aruco_dict_type=aruco_dict_type)

generate_aruco_board(board_width=3, board_height=4, marker_size_mm=marker_size_mm, 
                     marker_separation_mm=marker_separation_mm, screen_resolution=screen_resolution, 
                     screen_diagonal_inches=screen_diagonal_inches, aruco_dict_type=aruco_dict_type)
