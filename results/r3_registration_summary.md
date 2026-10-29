# R3 registration benchmark

Success = mean BIM-point displacement < 0.05 m. 10 offsets per stage per regime, 5 stages.

silent_fails = wrong AND not flagged (must be 0); flagged_fails = wrong but the method said so; false_alarms = right but flagged.

| method   | regime   |   trials |   success_rate |   silent_fails |   flagged_fails |   false_alarms |   median_err_m |   mean_secs |
|:---------|:---------|---------:|---------------:|---------------:|----------------:|---------------:|---------------:|------------:|
| icp_old  | gnss     |       50 |          0.000 |             50 |               0 |              0 |          2.518 |       7.112 |
| icp_old  | local    |       50 |          0.000 |             50 |               0 |              0 |         24.900 |      20.090 |
| plan     | gnss     |       50 |          0.980 |              0 |               1 |              1 |          0.008 |      54.409 |
| plan     | local    |       50 |          0.980 |              1 |               0 |              0 |          0.009 |     155.326 |

## Success rate by stage

| stage                            |   icp_old |   plan |
|:---------------------------------|----------:|-------:|
| 00_m1_fundering_structure        |      0.00 |   0.95 |
| 03_00_begane_grond_envelope      |      0.00 |   0.95 |
| 07_01_eerste_verdieping_interior |      0.00 |   1.00 |
| 11_03_derde_verdieping_structure |      0.00 |   1.00 |
| 15_04_dak_envelope               |      0.00 |   1.00 |
