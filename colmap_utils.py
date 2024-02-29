"""
COLMAP output consists of the files:

    points3D.txt: xyz world coordinates (followed by pairs or image ID & index of the associated observation in the corresponding line of images.txt)
    images.txt: extrinsic camera parameters (odd lines), point observations (even lines)
    cameras.txt: intrinsic camera parameters

The BAL data format is:

<num_cameras> <num_points> <num_observations>
<camera_index_1> <point_index_1> <x_1> <y_1>
...
<camera_index_num_observations> <point_index_num_observations> <x_num_observations> <y_num_observations>
<camera_1>
...
<camera_num_cameras>
<point_1>
...
<point_num_points>
"""

from pathlib import Path

# NOTE: right now the parameters are tailored to the OPENCV camera model
orderedOpenCvModelCameraKeys = ["camId", "model", "width", "height", "fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2"]
orderedImageKeys = ["imgId", "qw", "qx", "qy", "qz", "tx", "ty", "tz", "camId", "name"]

def read_colmap_output(colmapOutputDir):

    colmapOutputDir = Path(colmapOutputDir)
    cameras_txt = colmapOutputDir / "cameras.txt"
    images_txt = colmapOutputDir / "images.txt"
    points3d_txt = colmapOutputDir / "points3D.txt"

    cameras = {} # intrinsic camera parameters
    images = {} # extrinsic camera parameters
    points = {}

    # read cameras.txt
    lineNum=0
    linesOfHeaderCommentary = 3 # how many lines of commentary before the data
    with open(cameras_txt, "r") as file:
        while True:
            lineNum+=1
            line = file.readline()
            if not line: break
            if lineNum <= linesOfHeaderCommentary: continue
            camId, model, width, height, fx, fy, ox, oy, k1, k2, p1, p2 = line.split(" ")
            cameras[camId] = {
                "camId": camId,
                "model": model,
                "width": int(width), 
                "height": int(height),
                "fx": float(fx),
                "fy": float(fy),
                "cx": int(ox),
                "cy": int(oy), 
                "k1": float(k1),
                "k2": float(k2),
                "p1": float(p1),
                "p2": float(p2.strip())
            }

    # read images.txt
    lineNum=0
    linesOfHeaderCommentary = 4 # how many lines of commentary before the data
    with open(images_txt, "r") as inFile:
        while True:
            lineNum+=1
            line1 = inFile.readline() # camera extrinsics
            if not line1: break
            if lineNum <= linesOfHeaderCommentary: continue
            imageId, qw, qx, qy, qz, tx, ty, tz, camId, name = line1.split(" ")
            line2 = inFile.readline() # observations
            xyp = line2.split(" ")
            N = len(xyp) // 3
            # TODO: read observations

            #images[imgId] = {
            #    "name": name.strip(),
            images[name.strip()] = {
                "imgId": imageId,
                "name": name.strip(),
                # extrinsic camera parameters
                "qw": float(qw), 
                "qx": float(qx),
                "qy": float(qy),
                "qz": float(qz),
                "tx": float(tx),
                "ty": float(ty),
                "tz": float(tz),
                # intrinsic camera parameters
                "camId": camId
            }

    # read points3D.txt
    lineNum=0
    linesOfCommentary = 3 # how many lines of commentary before the data
    with open(points3d_txt, "r") as file:
        while True:
            lineNum+=1
            line = file.readline()
            if not line: break
            if lineNum <= linesOfCommentary: continue
            line = file.readline() # point info, unused
            pointId, x, y, z, r, g, b, error = line.split(" ")[:8] # the rest is (imageId, observationIdx) pairs
            track = line.split(" ")[8:] # the rest is (imageId, observationIdx) pairs
            points[pointId] = {
                "pointId": pointId,
                "x": float(x),
                "y": float(y),
                "z": float(z),
                "r": int(r),
                "g": int(g),
                "b": int(b),
                "error": float(error),
                "track": {imageId: int(obsIdx) for imageId, obsIdx in zip(track[::2], track[1::2])}
            }

    return images, cameras, points