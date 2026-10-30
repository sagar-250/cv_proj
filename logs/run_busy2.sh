set -e
PY=.venv/bin/python
$PY src/p3_render.py --sensor realistic --georef gnss --site busy > logs/busy2_p3.log 2>&1
$PY src/p4_match_bim.py > logs/busy2_p4.log 2>&1
$PY src/p4_visibility.py > logs/busy2_vis.log 2>&1
$PY src/p7_score.py --min-fraction 0 > logs/busy2_p7_off.log 2>&1
cp results/main_table.csv results/experiments/main_table_12_busy_late_misplaced_rule_off.csv
$PY src/p7_score.py > logs/busy2_p7.log 2>&1
cp results/main_table.csv results/experiments/main_table_13_busy_late_misplaced_rule_on.csv
rm -rf data/pointclouds_gnss_busy && mkdir -p data/pointclouds_gnss_busy
cp data/pointclouds/*.npz data/pointclouds_gnss_busy/ && cp -r data/matches data/pointclouds_gnss_busy/matches
echo BUSY2 DONE
