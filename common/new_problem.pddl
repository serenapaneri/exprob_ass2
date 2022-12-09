(define (problem task)
(:domain cluedo)
(:objects
    cluedo_robot - robot
    wp1 wp2 wp3 wp4 - waypoint
    home - home
    oracle - oracle
)
(:init

    (robot_at_home cluedo_robot home)





)
(:goal (and
    (game_finished)
))
)
