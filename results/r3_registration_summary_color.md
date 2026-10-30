# R3 registration benchmark

Success = mean BIM-point displacement < 0.05 m. 1 offsets per stage per regime, 6 stages.

silent_fails = wrong AND not flagged (must be 0); flagged_fails = wrong but the method said so; false_alarms = right but flagged.

| method              | regime   |   trials |   success_rate |   silent_fails |   flagged_fails |   false_alarms |   median_err_m |   mean_secs |
|:--------------------|:---------|---------:|---------------:|---------------:|----------------:|---------------:|---------------:|------------:|
| plan                | gnss     |        6 |          1.000 |              0 |               0 |              0 |          0.013 |     188.959 |
| plan                | local    |        6 |          1.000 |              0 |               0 |              0 |          0.012 |     367.422 |
| plan_color          | gnss     |        6 |          1.000 |              0 |               0 |              0 |          0.014 |     198.018 |
| plan_color          | local    |        6 |          1.000 |              0 |               0 |              0 |          0.013 |     350.222 |
| plan_color_rank     | gnss     |        6 |          1.000 |              0 |               0 |              0 |          0.015 |     168.490 |
| plan_color_rank     | local    |        6 |          1.000 |              0 |               0 |              0 |          0.012 |     328.255 |
| plan_color_wrongcat | gnss     |        6 |          1.000 |              0 |               0 |              0 |          0.013 |     263.282 |
| plan_color_wrongcat | local    |        6 |          1.000 |              0 |               0 |              0 |          0.013 |     433.774 |

## Success rate by stage

| stage                            |   plan |   plan_color |   plan_color_rank |   plan_color_wrongcat |
|:---------------------------------|-------:|-------------:|------------------:|----------------------:|
| 00_m1_fundering_structure        |   1.00 |         1.00 |              1.00 |                  1.00 |
| 01_m1_fundering_envelope         |   1.00 |         1.00 |              1.00 |                  1.00 |
| 03_00_begane_grond_envelope      |   1.00 |         1.00 |              1.00 |                  1.00 |
| 07_01_eerste_verdieping_interior |   1.00 |         1.00 |              1.00 |                  1.00 |
| 11_03_derde_verdieping_structure |   1.00 |         1.00 |              1.00 |                  1.00 |
| 15_04_dak_envelope               |   1.00 |         1.00 |              1.00 |                  1.00 |
