(define (problem task)
(:domain cluedo)
(:objects
    cluedo_robot - robot
    wp1 wp2 wp3 wp4 - waypoint
    home - home
    oracle - oracle
)
(:init



    (move_wp wp1 wp2)
    (move_wp wp2 wp3)
    (move_wp wp3 wp4)
    (move_wp wp4 wp1)


    (initialization)





)
(:goal (and
    (game_finished)
))
)
