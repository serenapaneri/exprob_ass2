(define (domain cluedo)

    (:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions :duration-inequalities)

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
        (gripper_on ?wp - waypoint)
        (hint_percieved)
        (complete_consistent_hypo)
        (game_finished)
    ) 
    
    (:durative-action away_home
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
		(at start (robot_at_wp ?obj ?wp))
		(at start (gripper_on ?wp)))
	:effect (and
	        (at end (not(gripper_on ?wp)))
	        (at end (hint_percieved)))
    )
     
    (:durative-action goto_waypoint
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
		(at start (hint_percieved)))
	:effect (and
	        (at end (complete_consistent_hypo))
	        (at start (not(hint_percieved))))
    )
    
    (:durative-action go_oracle
        :parameters (?obj - robot ?from - home ?to - oracle)
	:duration ( = ?duration 10)
	:condition (and
		(at start (robot_at_home ?obj ?from)))
	:effect (and
	        (at end (robot_at_oralce ?obj ?to))
		(at start (not(robot_at_home ?obj ?from))))
    )
    
    (:durative-action oracle
        :parameters (?obj - robot ?o - oracle)
	:duration ( = ?duration 5)
	:condition (and
		(at start (robot_at_oracle ?obj ?o))
		(at start (complete_consistent_hypo)))
	:effect (and
	        (at end (game_finished))
	        (at start (not(complete_consistent_hypo))))
    )
   
)
