set -e
PY=.venv/bin/python
$PY src/p3_render.py --sensor realistic --georef gnss > logs/rs3_p3.log 2>&1
$PY src/p4_match_bim.py > logs/rs3_p4.log 2>&1
$PY src/p4_visibility.py > logs/rs3_vis.log 2>&1
$PY src/p7_score.py --min-fraction 0 > logs/rs3_p7_off.log 2>&1
cp results/main_table.csv results/experiments/main_table_8_realistic_fixed_rule_off.csv
$PY src/p7_score.py > logs/rs3_p7.log 2>&1
cp results/main_table.csv results/experiments/main_table_9_realistic_fixed_rule_on.csv
rm -rf data/pointclouds_gnss_realistic && mkdir -p data/pointclouds_gnss_realistic
cp data/pointclouds/*.npz data/pointclouds_gnss_realistic/ && cp -r data/matches data/pointclouds_gnss_realistic/matches
echo REALISTIC DONE
$PY src/p3_render.py --sensor realistic --georef gnss --site busy > logs/busy_p3.log 2>&1
$PY src/p4_match_bim.py > logs/busy_p4.log 2>&1
$PY src/p4_visibility.py > logs/busy_vis.log 2>&1
$PY src/p7_score.py --min-fraction 0 > logs/busy_p7_off.log 2>&1
cp results/main_table.csv results/experiments/main_table_10_busy_realistic_rule_off.csv
$PY src/p7_score.py > logs/busy_p7.log 2>&1
cp results/main_table.csv results/experiments/main_table_11_busy_realistic_rule_on.csv
echo BUSY DONE
