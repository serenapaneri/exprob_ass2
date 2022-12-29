(define (problem task)
(:domain cluedo)
(:objects
    cluedo_robot - robot
    wp1 wp2 wp3 wp4 - waypoint
    home - home
    oracle - oracle
)
(:init


    (robot_at_oracle cluedo_robot oracle)

    (move wp1 wp2)
    (move wp2 wp3)
    (move wp3 wp4)
    (move wp4 wp1)

    (move_h home wp1)
    (move_h home wp2)
    (move_h home wp3)
    (move_h home wp4)







)
(:goal (and
    (game_finished)
))
)
