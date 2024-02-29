import numpy as np
import fire
from solve_coordinate_transform_via_linear_regression import solve_scale_rotation_translation, make_sRt

def run_test():

    # Example usage:

    # source points
    X = np.array([ # N x 3
        [1, 4, 3],
        [4, 7, 6],
        [7, 3, 9],
        [10, 1, 12],
        [13, 5, 15]
    ])

    s = 2.687

    R = np.array([
        [0.866, -0.5, 0],
        [0.5, 0.866, 0],
        [0, 0, 1]
    ])

    t = np.array([1, 2, 3])

    #sRt = make_sRt(s, R, t)

    Y = s * (R @ X.T).T + t

    print("X (source points):\n", X)
    print("s (scale): ", s)
    print("R (rotation matrix):\n", R)
    print("t (translation vector):\n", t)
    #print("Linear Transform Matrix:\n", sRt)
    print("Y = s * (R @ X.T).T + t  (target_points):\n", Y)

    # Find the linear transform
    R_hat, t_hat, s_hat = solve_scale_rotation_translation(X.T, Y.T)
    sRt_hat = make_sRt(s_hat, R_hat, t_hat)

    # compute Y_hat using s_hat, R_hat and t_hat
    Y_hat = (s_hat * R_hat @ X.T + t_hat).T

    print("Yhat (estimated target points):\n", Y_hat)
    print("max|Y - Yhat|: ", np.max(np.abs(Y_hat - Y)))

    # # compute Y_hat using linear transform
    # N = X.shape[0]
    # x = np.hstack((X, np.ones((N,1)))) # lifted source points
    # target_points = (sRt_hat  @ x.T).T[:,:3]

    # print("estimated target points:\n", Y_hat)
    # print("max error: ", np.max(Y_hat - Y))

if __name__ == "__main__":
    fire.Fire(run_test)