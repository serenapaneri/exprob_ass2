Number of literals: 13
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%] [120%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%] [120%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 12.000
b (11.000 | 5.000)b (10.000 | 20.002)b (9.000 | 25.003)b (8.000 | 30.004)b (7.000 | 35.005)b (6.000 | 40.006)b (5.000 | 45.007)b (4.000 | 50.008)b (3.000 | 55.009)b (2.000 | 60.010)b (1.000 | 65.011);;;; Solution Found
; States evaluated: 18
; Cost: 70.012
; Time 0.01
0.000: (start_game cluedo_robot home)  [5.000]
5.001: (leave_home cluedo_robot home wp1)  [10.000]
15.002: (move_arm cluedo_robot wp1)  [5.000]
15.003: (go_to_waypoint cluedo_robot wp1 wp2)  [10.000]
25.004: (move_arm cluedo_robot wp2)  [5.000]
25.005: (go_to_waypoint cluedo_robot wp2 wp3)  [10.000]
35.006: (move_arm cluedo_robot wp3)  [5.000]
35.007: (go_to_waypoint cluedo_robot wp3 wp4)  [10.000]
45.008: (move_arm cluedo_robot wp4)  [5.000]
45.009: (go_home cluedo_robot wp4 home)  [10.000]
55.010: (check_hypothesis cluedo_robot home)  [5.000]
55.011: (go_oracle cluedo_robot home oracle)  [10.000]
65.012: (oracle cluedo_robot oracle)  [5.000]
