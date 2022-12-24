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

    (hint_percieved wp1)
    (hint_percieved wp2)
    (hint_percieved wp3)
    (hint_percieved wp4)




)
(:goal (and
    (game_finished)
))
)
