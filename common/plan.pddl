Number of literals: 12
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 4.000
b (3.000 | 20.001)b (2.000 | 25.002)b (1.000 | 30.003);;;; Solution Found
; States evaluated: 6
; Cost: 35.004
; Time 0.00
0.000: (leave_oracle cluedo_robot oracle wp1)  [10.000]
10.001: (go_home cluedo_robot wp1 home)  [10.000]
20.002: (check_hypothesis cluedo_robot home)  [5.000]
20.003: (go_oracle cluedo_robot home oracle)  [10.000]
30.004: (oracle cluedo_robot oracle)  [5.000]
