(define (problem cluedo_game)
        (:domain cluedo)
        (:objects
            wp1 wp2 wp3 wp4 - waypoint
            home - home
            oracle - oracle
            cluedo_robot - robot
        )
          
        (:init
            (robot_at_home cluedo_robot home)
            (gripper_up)
            (not (hint_percieved wp1))
            (not (hint_percieved wp2))
            (not (hint_percieved wp3))
            (not (hint_percieved wp4))
        )

        (:goal (and
            (game_finished))
        )
)
