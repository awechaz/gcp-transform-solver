"""
Solves for the GCP transform.

Steps:
- Annotate 4 GCPs in a set of images (at least two observations of each GCP)
- Solve (via unconstrained optimization) for their COLMAP world coordinates 
- Solve (via linear regression) for the scaling, rotation, and translation operation that maps the known GCP coordinates to the solved COLMAP coordinates
- Test the transform on the annotations of an unseen image

BUGS:
- when the list of pointNames includes a valid GCP name that has no observations, 
  the solver outputs the point untouched (i.e. as its initialization)
  and shows no residual error, because there are no observations.
  this then throws off the 3D point cloud matching better eliminate 
  points without at least 2 observations.

TODO:


Another route...
- Fix a GCP and two image-plane observations of the GCP.
- Undistort the observations
- Solve for the COLMAP coordinates of the GCP by computing the "intersection" of the rays from these two observations (e.g. closest point on one line to the other) using the COLMAP camera model
- Repeat the above for N (4?) GCPs
- Solve for scaling by comparing the scale between any two GCPs in GCP coordinates vs. COLMAP coordinates
- Compute linear regression between the scaled GCP coordinates and their computed COLMAP coordinates 
"""

import fire
from config import *
from blueprints.nba.default import geometry

from solve_gcp_coordinates import solve_gcp_coordinates
from solve_coordinate_transform import solve_coordinate_transform

def read_solved_point_coordinates(pathToPointCoordinateSolution):
    """
    Reads just the solved point coordinates from the output from point_coordinate_solver.cc
    """
    with open(pathToPointCoordinateSolution, "r") as f:
        numImages, numCameras, numPoints, numObservations = map(int, f.readline().strip().split(" "))
        numLinesToSkip = numObservations + numImages + numCameras 
        for i in range(numLinesToSkip):
            f.readline()
        points = [list(map(float, f.readline().strip().split(" "))) for i in range(numPoints)]
    return points

def solve_gcp_transform(colmapOutputDir, *pointNames):

    # Solve for GCP coordinates in COLMAP system
    pathToPointCoordinateSolution = solve_gcp_coordinates(colmapOutputDir, *pointNames)
    solvedXYZs = read_solved_point_coordinates(pathToPointCoordinateSolution)

    # Write GCP transform problem
    pathToGcpTransformProblem = TMP_DIR / "gcp_transform_problem.txt"
    with open(pathToGcpTransformProblem, "w") as f:
        for pointName, xyz_hat in zip(pointNames, solvedXYZs):
            xhat, yhat, zhat = xyz_hat
            x, y, z = geometry["points"][pointName]["xyz"]
            f.write(f"{x} {y} {z} {xhat} {yhat} {zhat}\n")

    print("\nGCP Transform problem has been written to:\n", pathToGcpTransformProblem)

    print("Solving GCP transform problem:\n", pathToGcpTransformProblem)
    pathToGcpTransformSolution = TMP_DIR / "gcp_transform_solution.txt"
    solve_coordinate_transform(pathToGcpTransformProblem, pathToGcpTransformSolution)
    print("\nGCP Transform solution has been written to:\n", pathToGcpTransformSolution)

if __name__ == "__main__":
    fire.Fire(solve_gcp_transform)