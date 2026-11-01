set -e
PY=.venv/bin/python
S=07_01_eerste_verdieping_interior
$PY src/sfm_experiment.py --stage $S --step render > logs/sfm_render_07.log 2>&1
$PY src/sfm_experiment.py --stage $S --step colmap > logs/sfm_colmap_07.log 2>&1
$PY src/sfm_experiment.py --stage $S --step cloud > logs/sfm_cloud_07.log 2>&1
$PY src/sfm_experiment.py --stage $S --step ideal > logs/sfm_ideal_07.log 2>&1
export RUN_TAG=sfm
$PY src/p4_match_bim.py --stage $S > logs/sfm_p4_07.log 2>&1
$PY src/p4_visibility.py --stage $S > logs/sfm_vis_07.log 2>&1
$PY src/p7_score.py --calib "" > logs/sfm_p7_07.log 2>&1
export RUN_TAG=sfm_ideal
$PY src/p4_match_bim.py --stage $S > logs/sfm_ideal_p4_07.log 2>&1
$PY src/p4_visibility.py --stage $S > logs/sfm_ideal_vis_07.log 2>&1
$PY src/p7_score.py --calib "" > logs/sfm_ideal_p7_07.log 2>&1
echo SFM07 DONE
