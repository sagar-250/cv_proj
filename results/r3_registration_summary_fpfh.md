# R3 registration benchmark

Success = mean BIM-point displacement < 0.05 m. 2 offsets per stage per regime, 5 stages.

silent_fails = wrong AND not flagged (must be 0); flagged_fails = wrong but the method said so; false_alarms = right but flagged.

| method   | regime   |   trials |   success_rate |   silent_fails |   flagged_fails |   false_alarms |   median_err_m |   mean_secs |
|:---------|:---------|---------:|---------------:|---------------:|----------------:|---------------:|---------------:|------------:|
| fpfh     | gnss     |       10 |          0.400 |              4 |               2 |              0 |          1.016 |      36.333 |
| fpfh     | local    |       10 |          0.300 |              2 |               5 |              0 |         18.115 |      23.115 |

## Success rate by stage

| stage                            |   fpfh |
|:---------------------------------|-------:|
| 00_m1_fundering_structure        |   0.00 |
| 03_00_begane_grond_envelope      |   0.00 |
| 07_01_eerste_verdieping_interior |   0.00 |
| 11_03_derde_verdieping_structure |   0.75 |
| 15_04_dak_envelope               |   1.00 |
