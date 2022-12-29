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
            (game_finished))
        )
)
