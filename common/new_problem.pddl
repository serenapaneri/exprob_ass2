(define (problem task)
(:domain cluedo)
(:objects
    cluedo_robot - robot
    wp1 wp2 wp3 wp4 - waypoint
    home - home
    oracle - oracle
)
(:init
    (robot_at_wp cluedo_robot wp4)



    (move wp1 wp2)
    (move wp2 wp3)
    (move wp3 wp4)
    (move wp4 wp1)







)
(:goal (and
    (game_finished)
))
)
