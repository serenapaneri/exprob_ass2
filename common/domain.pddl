(define (domain cluedo)

    (:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions :adl :duration-inequalities :equality)

    (:types 
        robot
        waypoint
        home
        oracle
    )
    
    (:predicates
        (robot_at_wp ?obj - robot ?wp - waypoint)
        (robot_at_home ?obj - robot ?h - home)
        (robot_at_oracle ?obj -robot ?o - oracle)
        (move_wp ?from ?to - waypoint)
        (move_h ?from - home ?to - waypoint)
        (move_o ?from - oracle ?to - waypoint)
        (hint_percieved ?wp - waypoint)
        (initialization)
        (gripper_up)
        (complete_hypo)
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
	        (at start (robot_at_home ?obj ?from))
	        (at start (move_h ?from ?to)))
	:effect (and
	        (at end (robot_at_wp ?obj ?to))
	        (at end (gripper_up))
	        (at start (not(robot_at_home ?obj ?from))))
    )
    
    (:durative-action move_arm
        :parameters (?obj - robot ?wp - waypoint)
	:duration ( = ?duration 5)
	:condition (and
		(at start (robot_at_wp ?obj ?wp))
		(at start (gripper_up)))
	:effect (and
	        (at end (hint_percieved ?wp))
	        (at start (not(gripper_up))))
    )
     
    (:durative-action go_to_waypoint
	:parameters (?obj - robot ?from - waypoint ?to - waypoint)
	:duration ( = ?duration 10)
	:condition (and
		(at start (robot_at_wp ?obj ?from))
		(at start (move_wp ?from ?to)))
	:effect (and
	        (at end (robot_at_wp ?obj ?to))
	        (at end (gripper_up))
		(at start (not(robot_at_wp ?obj ?from))))
    )
    
    (:durative-action check_complete
	:parameters (?obj - robot ?wp - waypoint)
	:duration ( = ?duration 5)
	:condition (and
		(at start (robot_at_wp ?obj ?wp))
		(at start (hint_percieved ?wp)))
	:effect (and
	        (at end (complete_hypo))
		(at start (not(hint_percieved ?wp))))
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
    
    (:durative-action check_consistency
        :parameters (?obj - robot ?h - home)
	:duration ( = ?duration 5)
	:condition (and
	        (at start (robot_at_home ?obj ?h))
		(at start (complete_hypo)))
	:effect (and
	        (at end (complete_consistent_hypo))
	        (at start (not(complete_hypo))))
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
	        (at end (game_finished))
	        (at start (not(complete_consistent_hypo))))
    )
    
    (:durative-action leave_oracle
	:parameters (?obj - robot ?from - oracle ?to - waypoint)
	:duration ( = ?duration 10)
	:condition (and
		(at start (robot_at_oracle ?obj ?from))
		(at start (move_o ?from ?to)))
	:effect (and
	        (at end (robot_at_wp ?obj ?to))
		(at start (not(robot_at_oracle ?obj ?from))))
    )
   
)
