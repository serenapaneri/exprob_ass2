(define (problem cluedo_game)
          (:domain cluedo)
          (:objects
              wp1 wp2 wp3 wp4 home oracle_room - waypoint
              cluedo_robot - robot
          )
          
          (:init
              (robot_at robot home)
              (gripper on home)
              (visited home)
              (not(visited wp1))
              (not(visited wp2))
              (not(visited wp3))
              (not(visited wp4))
              (not(visited oracle_room))
          )

          (:goal
              (game_finished)
          )
)
