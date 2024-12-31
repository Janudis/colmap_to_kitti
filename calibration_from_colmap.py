import os
import numpy as np
import cv2
import pickle
import open3d as o3d
import sys
sys.path.append('/media/harddrive/colmap/scripts/python')
from read_write_model import read_model, qvec2rotmat
import matplotlib.pyplot as plt
import trimesh

# Paths to the files
model_path = '/media/harddrive/new_dataset_colmap/github_script/colmap_output/sparse/0'  # Path to the model directory
image_dir = '/media/harddrive/new_dataset_colmap/github_script/images'  # Directory containing your images
projections_dir = '/media/harddrive/new_dataset_colmap/github_script/projected_images_sfm'  # Directory to save projected images
output_dir = '/media/harddrive/new_dataset_colmap/github_script/calib'  # Directory to save KITTI calibration files
# Ensure output directory exists
os.makedirs(projections_dir, exist_ok=True)
os.makedirs(output_dir, exist_ok=True)

# Read data
cameras, images, _ = read_model(path=model_path, ext='.bin')
# Read the dense point cloud
pcd = o3d.io.read_point_cloud('/media/harddrive/new_dataset_colmap/github_script/colmap_output/dense/processed_fused.ply')
# Get the point coordinates as a NumPy array
points3D_xyz = np.asarray(pcd.points)  # Shape (N, 3)
frustums = []
# Iterate over each image
for image_id, image in images.items():
    # Get image file path
    image_name = image.name
    image_path = os.path.join(image_dir, image_name)
    output_image_path = os.path.join(projections_dir, image_name)
    # Check if image exists
    if not os.path.isfile(image_path):
        print(f"Image {image_name} not found in {image_dir}")
        continue
    # Read image
    img = cv2.imread(image_path)
    if img is None:
        print(f"Failed to read image {image_name}")
        continue
    # Get camera parameters
    camera = cameras[image.camera_id]
    params = camera.params
    width = camera.width
    height = camera.height
    model = camera.model
    # Build intrinsic matrix and distortion parameters
    if model == 'SIMPLE_PINHOLE':
        f = params[0]
        cx = params[1]
        cy = params[2]
        K = np.array([[f, 0, cx],
                      [0, f, cy],
                      [0, 0, 1]])
        distortion = None
    elif model == 'PINHOLE':
        fx = params[0]
        fy = params[1]
        cx = params[2]
        cy = params[3]
        K = np.array([[fx, 0, cx],
                      [0, fy, cy],
                      [0, 0, 1]])
        distortion = None
    elif model == 'SIMPLE_RADIAL':
        f = params[0]
        cx = params[1]
        cy = params[2]
        k = params[3]
        K = np.array([[f, 0, cx],
                      [0, f, cy],
                      [0, 0, 1]])
        distortion = np.array([k])
    elif model == 'RADIAL':
        f = params[0]
        cx = params[1]
        cy = params[2]
        k1 = params[3]
        k2 = params[4]
        K = np.array([[f, 0, cx],
                      [0, f, cy],
                      [0, 0, 1]])
        distortion = np.array([k1, k2])
    else:
        print(f"Camera model {model} is not supported.")
        continue
    # Get rotation and translation
    R = qvec2rotmat(image.qvec)
    t = image.tvec.reshape(3, 1)
    X_world = points3D_xyz.T  # Shape (3, N)
    X_cam = R @ X_world + t  # Shape (3, N)
    # Transform points to normalized image coordinates
    x_cam = X_cam[0, :] / X_cam[2, :]
    y_cam = X_cam[1, :] / X_cam[2, :]
    # Apply distortion if necessary
    if distortion is not None:
        r2 = x_cam**2 + y_cam**2
        if model == 'SIMPLE_RADIAL':
            k = distortion[0]
            radial = 1 + k * r2
        elif model == 'RADIAL':
            k1, k2 = distortion
            radial = 1 + k1 * r2 + k2 * r2**2
        else:
            radial = 1
        x_distorted = x_cam * radial
        y_distorted = y_cam * radial
    else:
        x_distorted = x_cam
        y_distorted = y_cam
    # Convert to pixel coordinates
    x_proj = x_distorted * K[0, 0] + K[0, 2]
    y_proj = y_distorted * K[1, 1] + K[1, 2]
    proj_points = np.vstack([x_proj, y_proj, np.ones_like(x_proj)])
    # Filter points within image bounds and in front of the camera
    valid_mask = (proj_points[0, :] >= 0) & (proj_points[0, :] < width) & \
                 (proj_points[1, :] >= 0) & (proj_points[1, :] < height) & \
                 (X_cam[2, :] > 0)
    x_proj_valid = proj_points[0, valid_mask]
    y_proj_valid = proj_points[1, valid_mask]
    # Prepare the figure
    plt.figure(figsize=(10, 8))
    plt.imshow(img[:, :, ::-1])  # Convert BGR to RGB for correct colors in plt
    plt.scatter(x_proj_valid, y_proj_valid, s=5, c='red')  # Plot the projections
    plt.axis('off')  # Remove axes
    # Save the figure first
    output_image_path_matplotlib = os.path.join(projections_dir, f"mpl_{image_name}")  # Save with a prefix
    plt.savefig(output_image_path_matplotlib, bbox_inches='tight', pad_inches=0, dpi=300)
    print(f"Projected points saved using matplotlib to '{output_image_path_matplotlib}'")
    # Show the figure afterward
    # plt.show()
    plt.close()

    # Construct Extrinsic Matrix [R | T]
    extrinsic_matrix = np.hstack((R, t))  # [R | T] -> 3x4
    extrinsic_matrix_4x4 = np.vstack((extrinsic_matrix, [0, 0, 0, 1]))  # 4x4 matrix
    # Construct Projection Matrix P2 = [K | 0]
    P2 = np.hstack((K, np.zeros((3, 1))))  # [3x3 | 3x1] -> 3x4
    P2_flat = P2.flatten()
    calibration_content = f"P2: {' '.join([f'{value:.12e}' for value in P2_flat])}\n"
    # Add other parameters with zero placeholders
    calibration_content += "P0: " + "0.000000000000e+00 " * 12 + "\n"
    calibration_content += "P1: " + "0.000000000000e+00 " * 12 + "\n"
    calibration_content += "P3: " + "0.000000000000e+00 " * 12 + "\n"
    calibration_content += "R0_rect: 1.000000000000e+00 0.000000000000e+00 0.000000000000e+00 " \
                           "0.000000000000e+00 1.000000000000e+00 0.000000000000e+00 " \
                           "0.000000000000e+00 0.000000000000e+00 1.000000000000e+00\n"
    calibration_content += "Tr_velo_to_cam: " + "0.000000000000e+00 " * 12 + "\n"
    calibration_content += "Tr_imu_to_velo: " + "0.000000000000e+00 " * 12 + "\n"
    # Write calibration file for each image
    output_file_path = os.path.join(output_dir, f"{int(image_id - 1):06d}.txt")
    with open(output_file_path, 'w') as calib_file:
        calib_file.write(calibration_content)
    print(f"Calibration files saved in {output_dir}")