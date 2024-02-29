"""
Given a set of 3D world coordinates in two frames of reference, solves the linear regression problem mapping one to the other.
"""

import numpy as np
import fire

def solve_scale_rotation_translation(A, B):
    # Input: expects 3xN matrix of points
    # Returns s, R, t
    # s = scale factor
    # R = 3x3 rotation matrix
    # t = 3x1 column vector

    assert A.shape == B.shape

    num_rows, num_cols = A.shape
    if num_rows != 3:
        raise Exception(f"matrix A is not 3xN, it is {num_rows}x{num_cols}")

    num_rows, num_cols = B.shape
    if num_rows != 3:
        raise Exception(f"matrix B is not 3xN, it is {num_rows}x{num_cols}")

    # find mean column wise
    centroid_A = np.mean(A, axis=1)
    centroid_B = np.mean(B, axis=1)

    # ensure centroids are 3x1
    centroid_A = centroid_A.reshape(-1, 1)
    centroid_B = centroid_B.reshape(-1, 1)

    # subtract mean
    Am = A - centroid_A
    Bm = B - centroid_B

    # compute scaling
    s = np.mean( np.linalg.norm(Bm, axis=0) / np.linalg.norm(Am, axis=0) )
    Am *= s
    centroid_A *= s

    H = Am @ np.transpose(Bm)

    # sanity check
    #if linalg.matrix_rank(H) < 3:
    #    raise ValueError("rank of H = {}, expecting 3".format(linalg.matrix_rank(H)))

    # find rotation
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # special reflection case
    if np.linalg.det(R) < 0:
        print("det(R) < R, reflection detected!, correcting for it ...")
        Vt[2,:] *= -1
        R = Vt.T @ U.T

    t = -R @ centroid_A + centroid_B

    return R, t, s

def make_sRt(s, R, t):
    sRt = np.eye(4)
    sRt[:3,:3] = s * R
    sRt[:3, 3] = t.flatten()
    return sRt

def solve_coordinate_transform(inputFile, outputFile):

    # Open the file
    X = []
    Y = []
    with open(inputFile, 'r') as file:
        # Read lines from the file
        lines = file.readlines()
        for line in lines:
            xyz1_xyz2 = list( map(float, line.strip().split()) )
            xyz1 = xyz1_xyz2[:3]
            xyz2 = xyz1_xyz2[3:]
            X.append(xyz1)
            Y.append(xyz2)
    X = np.array(X)
    Y = np.array(Y)

    print("X (source points):\n", X)
    print("\nY (target points):\n", Y)

    R_hat, t_hat, s_hat = solve_scale_rotation_translation(X.T, Y.T)
    #sRt_hat = make_sRt(s_hat, R_hat, t_hat)
    Y_hat = (s_hat * R_hat @ X.T + t_hat).T # compute Y_hat using s_hat, R_hat and t_hat

    print("s_hat (est. scale): ", s_hat)
    print("R_hat (est. rotation matrix):\n", R_hat)
    print("t_hat (est. translation vector):\n", t_hat)
    #print("Linear Transform Matrix:\n", sRt)
    print("Y_hat = s_hat * (R_hat @ X.T).T + t_hat  (target_points):\n", Y_hat)

    print("max|Y - Yhat|: ", np.max(np.abs(Y_hat - Y)))

    # Convert to format C++ can read
    from scipy.spatial.transform import Rotation

    # # Convert rotation matrix to Rodrigues vector
    # r = Rotation.from_matrix(R_hat)
    # rod = r.as_mrp()

    # write transform to file
    t_hat = t_hat.ravel()
    with open(outputFile, "w") as f:
        #f.write(f"{s_hat} {rod[0]} {rod[1]} {rod[2]} {t_hat[0]} {t_hat[1]} {t_hat[2]}")
        f.write(f"{s_hat} {R_hat[0][0]} {R_hat[0][1]} {R_hat[0][2]} {R_hat[1][0]} {R_hat[1][1]} {R_hat[1][2]} {R_hat[2][0]} {R_hat[2][1]} {R_hat[2][2]} {t_hat[0]} {t_hat[1]} {t_hat[2]}")

    print(f"\nC++ input written to {outputFile}")


if __name__ == "__main__":
    fire.Fire(solve_coordinate_transform)