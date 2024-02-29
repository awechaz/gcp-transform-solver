# gcp-transform-solver
Solves for the coordinate transform to COLMAP world coordinates implied by annotated GCPs.

python solve_gcp_transform.py --colmapOutputDir=~/awecom/gcp-transform-solver/data/colmap tl_ftline_at_laneline_oo tr_ftline_at_laneline_oo tl_court_corner_i tr_court_corner_i t_divline_at_sideline b_divline_at_sideline bl_ftline_at_laneline_oo br_ftline_at_laneline_oo
./build/bin/test_gcp_transform tmp/gcp_transform_solution.txt
