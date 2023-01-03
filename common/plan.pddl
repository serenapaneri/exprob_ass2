Number of literals: 15
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%] [120%] [130%] [140%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%] [120%] [130%] [140%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 7.000
b (6.000 | 5.000)b (5.000 | 20.002)b (4.000 | 25.003)b (3.000 | 30.004)b (2.000 | 35.005)b (1.000 | 40.006);;;; Solution Found
; States evaluated: 10
; Cost: 45.007
; Time 0.01
0.000: (start_game cluedo_robot home)  [5.000]
5.001: (leave_home cluedo_robot home wp1)  [10.000]
15.002: (move_arm cluedo_robot wp1)  [5.000]
20.003: (check_complete cluedo_robot wp1)  [5.000]
20.004: (go_home cluedo_robot wp1 home)  [10.000]
30.005: (check_consistency cluedo_robot home)  [5.000]
30.006: (go_oracle cluedo_robot home oracle)  [10.000]
40.007: (oracle cluedo_robot oracle)  [5.000]
