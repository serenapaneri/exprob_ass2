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
        )

        (:goal (and
            (game_finished))
        )
)
