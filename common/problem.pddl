(define (problem cluedo_game)
        (:domain cluedo)
        (:objects
            wp1 wp2 wp3 wp4 - waypoint
            home - home
            oracle - oracle
            cluedo_robot - robot
        )
          
        (:init
            (initialization)
            (move_wp wp1 wp2)
            (move_wp wp2 wp3)
            (move_wp wp3 wp4)
            (move_wp wp4 wp1)
            (move_h home wp1)
            (move_h home wp2)
            (move_h home wp3)
            (move_h home wp4)
            (move_o oracle wp1)
            (move_o oracle wp2)
            (move_o oracle wp3)
            (move_o oracle wp4)
        )

        (:goal (and
            (game_finished))
        )
)
