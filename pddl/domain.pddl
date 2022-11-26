(define (domain cluedo_robot)

    (:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions  :negative-preconditions :duration-inequalities)

    (:types 
        robot
        oracle
        waypoint
        home
    )
    
    (:predicates
        (robot_at ?obj - robot ?wp -waypoint)
        (robot_at ?obj - robot ?loc - home)
        (visited ?wp - waypoint)
        (move_to ?from ?to - waypoint)
        (check_hypo)
        (try_guess)
    )
    
    (:durative-action goto_waypoint
	:parameters (?obj - robot ?from ?to - waypoint)
	:duration ( = ?duration 60)
	:condition (and
		(at start (robot_at ?v ?from))
		(at start (localised ?v))
		(over all (undocked ?v)))
	:effect (and
		(at end (visited ?to))
		(at end (robot_at ?v ?to))
		(at start (not (robot_at ?v ?from))))
)
)
