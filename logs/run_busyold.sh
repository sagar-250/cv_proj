set -e
PY=.venv/bin/python
export RUN_TAG=busyold
$PY src/p4_match_bim.py --registration icp_old > logs/busyold_p4.log 2>&1
$PY src/p4_visibility.py > logs/busyold_vis.log 2>&1
$PY src/p7_score.py --calib "" > logs/busyold_p7.log 2>&1
echo BUSYOLD DONE
