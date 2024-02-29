"""
Solves for the 3D coordinates of a GCP in the COLMAP world coordinate system.
"""

import fire
import os
import json
from pathlib import Path
import random
import numpy as np

from config import *
from colmap_utils import read_colmap_output
from blueprints.nba.default import geometry

def generate_point_coordinate_problem(colmapOutputDir, *pointsToInclude):
    """
    Takes the .json files containing annotated GCPs for a given image in the format:

    {
        pointId_1: [x_1, y_1]
    },
    ...
    {
        pointId_N: [x_N, y_N]
    },
    
    and combines with COLMAP output to compose a .txt file in the format:

    imageId_1, camId_1, pointId_1, x_1, y_1
    ...
    imageId_N, camId_N, pointId_N, x_N, y_N

    where imageId, camId, and pointId are all re-assigned starting from 0. 
    """
    colmapOutputDir = Path(colmapOutputDir).expanduser()
    gcpFilepaths = list( (colmapOutputDir / "images").glob('*.json') )
    outPath = TMP_DIR / "point_coordinate_problem.txt"

    images, cameras, points = read_colmap_output(colmapOutputDir)

    numImages = 0 # extrinsic camera parameters
    numCameras = 0 # intrinsic camera parameters
    numPoints = 0 # landmarks with 3D world coordinates
    numObservations = 0 # obsrvations of landmarks with image-plane coordinates

    pointDict = {pointName: str(i) for i, pointName in enumerate(pointsToInclude)} # we want the indexing of points to match the ordering in the pointsToInclude list so we know which is which
    numPoints = len(pointsToInclude)
    cameraDict = {} # COLMAP camera id -> camera index
    imageDict = {} # COLMAP camera id -> camera index
    outLines = []
    for fpath in gcpFilepaths:
        imageId = numImages
        imageName = fpath.with_suffix(".JPG").name
        if imageName not in imageDict:
            imageDict[imageName] = numImages
            numImages += 1
        imageId = imageDict[imageName]

        cameraName = images[imageName]["camId"] # the COLMAP cam id
        if cameraName not in cameraDict:
            cameraDict[cameraName] = numCameras
            numCameras += 1
        cameraId = cameraDict[cameraName]

        with open(fpath, "r") as inFile:
            J = json.load(inFile)


        for pointName in J.keys():
            if pointName not in pointsToInclude:
                continue
            pointId = pointDict[pointName]

            x, y = J[pointName]
            outLine = f"{imageId} {pointId} {x} {y}\n"
            outLines.append(outLine)
            numObservations += 1

    with open(outPath, "w") as outFile:
        outFile.write(f"{numImages} {numCameras} {numPoints} {numObservations}\n")
        for outLine in outLines:
            outFile.write(outLine)

        imageDictInv = {val: key for key, val in imageDict.items()}
        cameraDictInv = {val: key for key, val in cameraDict.items()}
        pointDictInv = {val: key for key, val in pointDict.items()}

        # print extrinsic camera parameters 
        for imageId in range(numImages):
            imageName = imageDictInv[imageId]
            image = images[imageName]
            camId = cameraDict[image['camId']]
            outLine = f"{camId} {image['qw']} {image['qx']} {image['qy']} {image['qz']} {image['tx']} {image['ty']} {image['tz']}\n"
            #outLine = "\n".join(outLine.split(" "))
            outFile.write(outLine)

        # print intrinsic camera parameters
        for cameraId in range(numCameras):
            cameraName = cameraDictInv[cameraId]
            camera = cameras[cameraName]
            outLine = f"{camera['width']} {camera['height']} {camera['fx']} {camera['fy']} {camera['cx']} {camera['cy']} {camera['k1']} {camera['k2']} {camera['p1']} {camera['p2']}\n"
            #outLine = "\n".join(outLine.split(" "))
            outFile.write(outLine)

        # print initial point parameters
        XYZ = np.array([[point["x"], point["y"], point["z"]] for point in points.values()])
        x_avg, y_avg, z_avg = np.median(XYZ, axis=0)
        for pointId in range(numPoints):
            outLine = f"{x_avg} {y_avg} {z_avg}\n"
            #outLine = "\n".join(outLine.split(" "))
            outFile.write(outLine)

        print("Point coordinate problem has been written to:")
        print(outPath)

        return outPath

        # TODO: initialize via intersection of lines from two observations? 
        # Would need to map the equation of the ray from each camera into the COLMAP world coordinate system, 
        # then find closest point on one to the other in those coordinates

def solve_gcp_coordinates(colmapOutputDir, *pointNames):

    for pointName in pointNames:
        if pointName not in geometry["points"].keys():
            print(f"{pointName} is not a known GCP label")
            exit(1)

    pathToPointCoordinateProblem = generate_point_coordinate_problem(
        colmapOutputDir,
        *pointNames
    )

    pathToPointCoordinateSolution = TMP_DIR / "point_coordinate_solution.txt"
    cmd = f"{BINARIES_DIR}/point_coordinate_solver {str(pathToPointCoordinateProblem)} {str(pathToPointCoordinateSolution)}"
    os.system(cmd)

    print("\nPoint coordinate solution has been written to:")
    print(pathToPointCoordinateSolution)

    return pathToPointCoordinateSolution

if __name__ == "__main__":
    fire.Fire(solve_gcp_coordinates)