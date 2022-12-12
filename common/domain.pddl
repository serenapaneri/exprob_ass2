(define (domain cluedo)

    (:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions :adl :duration-inequalities)

    (:types 
        robot
        waypoint
        home
        oracle
    )
    
    (:predicates
        (robot_at_wp ?obj - robot ?wp -waypoint)
        (robot_at_home ?obj - robot ?h - home)
        (robot_at_oracle ?obj -robot ?o - oracle)
        (hint_percieved ?wp - waypoint)
        (initialization)
        (complete_consistent_hypo)
        (game_finished)
    )
    
    (:durative-action start_game
	:parameters (?obj - robot ?h - home)
	:duration ( = ?duration 5)
	:condition (and
	        (at start (initialization)))
	:effect (and
	        (at end (robot_at_home ?obj ?h))
	        (at start (not(initialization))))
    )
    
    (:durative-action leave_home
	:parameters (?obj - robot ?from - home ?to - waypoint)
	:duration ( = ?duration 10)
	:condition (and
	        (at start (robot_at_home ?obj ?from)))
	:effect (and
	        (at end (robot_at_wp ?obj ?to))
	        (at start (not(robot_at_home ?obj ?from))))
    )
    
    (:durative-action move_arm
        :parameters (?obj - robot ?wp - waypoint)
	:duration ( = ?duration 5)
	:condition (and
		(at start (robot_at_wp ?obj ?wp)))
	:effect (and
	        (at end (hint_percieved ?wp)))
    )
     
    (:durative-action go_to_waypoint
	:parameters (?obj - robot ?from ?to - waypoint)
	:duration ( = ?duration 10)
	:condition (and
		(at start (robot_at_wp ?obj ?from)))
	:effect (and
	        (at end (robot_at_wp ?obj ?to))
		(at start (not(robot_at_wp ?obj ?from))))
    )
    
    (:durative-action go_home
        :parameters (?obj - robot ?from - waypoint ?to - home)
	:duration ( = ?duration 10)
	:condition (and
		(at start (robot_at_wp ?obj ?from)))
	:effect (and
	        (at end (robot_at_home ?obj ?to))
		(at start (not(robot_at_wp ?obj ?from))))
    )
    
    (:durative-action check_hypothesis
        :parameters (?obj - robot ?h - home)
	:duration ( = ?duration 5)
	:condition (and
	        (at start (robot_at_home ?obj ?h))
		(at start (forall (?wp - waypoint) (hint_percieved ?wp))))
	:effect (and
	        (at end (complete_consistent_hypo))
	       )
    )
    
    (:durative-action go_oracle
        :parameters (?obj - robot ?from - home ?to - oracle)
	:duration ( = ?duration 10)
	:condition (and
		(at start (robot_at_home ?obj ?from)))
	:effect (and
	        (at end (robot_at_oracle ?obj ?to))
		(at start (not(robot_at_home ?obj ?from))))
    )
    
    (:durative-action oracle
        :parameters (?obj - robot ?o - oracle)
	:duration ( = ?duration 5)
	:condition (and
		(at start (robot_at_oracle ?obj ?o))
		(at start (complete_consistent_hypo)))
	:effect (and
	        (at end (game_finished)))
    )
    
    (:durative-action leave_oracle
	:parameters (?obj - robot ?from - oracle ?to - waypoint)
	:duration ( = ?duration 10)
	:condition (and
		(at start (robot_at_oracle ?obj ?from)))
	:effect (and
	        (at end (robot_at_wp ?obj ?to))
		(at start (not(robot_at_oracle ?obj ?from))))
    )
   
)
