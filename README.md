# 3D Reconstruction and KITTI Calibration Generation

This repository demonstrates how to:
1. Reconstruct a 3D model (mesh + sparse/dense point clouds) of an object (in this example, a motorcycle) using [COLMAP](https://colmap.github.io/).
2. Generate calibration files in [KITTI](http://www.cvlibs.net/datasets/kitti/) format.
3. Validate the generated calibration by projecting 3D points onto the original images.

---

## Table of Contents
- [Overview](#overview)
- [Project Structure](#project-structure)
- [Dependencies and Installation](#dependencies-and-installation)
- [Usage](#usage)
  - [1. Run COLMAP for Reconstruction](#1-run-colmap-for-reconstruction)
  - [2. Prepare the Project Folder](#2-prepare-the-project-folder)
  - [3. Run the Calibration Script](#3-run-the-calibration-script)
---

## Overview
This project uses COLMAP to perform a 3D reconstruction (sparse and dense) from a set of images. Once the reconstruction is done, we read COLMAP’s output (camera models, poses, and point clouds) and create:
- **Projected images**: Visualizing the 3D points projected onto each original image.
- **Calibration files in KITTI format**: Specifically, the intrinsic and extrinsic parameters are formatted according to KITTI standards.

---

## Project Structure
- The `sparse/0/` folder contains COLMAP’s binary model files (`cameras.bin`, `images.bin`, and `points3D.bin`).  
- The `dense/` folder contains the dense point cloud (`fused.ply` or `processed_fused.ply`).  
- `images/` must contain the exact same images used by COLMAP.  
- `calib/` and `projected_images_sfm/` are output directories created by the script.

---

## Dependencies and Installation

1. **COLMAP** (for the 3D reconstruction).  
   - [Official Installation Instructions](https://colmap.github.io/install.html)

2. **Python** (3.6+ recommended).
3. **Python packages**:
   - `numpy`
   - `opencv-python` (as `cv2`)
   - `open3d`
   - `matplotlib`
   - `trimesh` (optional if you want to handle mesh objects)
   - (Optional) `pickle` (part of the Python standard library)

    COLMAP Python scripts:
        Make sure you can import the read_write_model from COLMAP.
        By default, the script in this repository uses:

        sys.path.append('/media/harddrive/colmap/scripts/python')
        from read_write_model import read_model, qvec2rotmat

        Update the sys.path.append(...) line if your COLMAP Python scripts are located elsewhere.

Usage
1. Run COLMAP for Reconstruction

    Capture Images:
        Take your images of the object vertically (portrait orientation) on your phone.
        This ensures that the image dimensions match the sensor dimensions during calibration.
        Place these images in the images/ directory.

    Use COLMAP:
        Create a COLMAP project and run:
            Feature extraction
            Feature matching
            Sparse reconstruction
            Dense reconstruction

    Resulting Folders:
        After COLMAP finishes, you should have:
            colmap_output/sparse/0/ containing .bin files.
            colmap_output/dense/ containing .ply files.

2. Prepare the Project Folder

    The script reads from colmap_output/sparse/0/ and colmap_output/dense/processed_fused.ply by default.

    If your files differ, update the paths inside calibration_from_colmap.py.

4. Run the Calibration Script

    Navigate to the folder containing calibration_from_colmap.py (e.g., github_script/):

    cd /path/to/github_script
    python calibration_from_colmap.py

    The script will:
        Read the COLMAP output data (camera params, extrinsics, point cloud).
        Project 3D points onto each image and save them in projected_images_sfm/.
        Write KITTI-format calibration files into the calib/ directory.

