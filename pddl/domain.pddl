(define (domain cluedo)

    (:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions :duration-inequalities)

    (:types 
        robot
        waypoint
    )
    
    (:predicates
        (robot_at ?obj - robot ?wp -waypoint)
        (visited ?wp - waypoint)
        (gripper_on ?wp - waypoint)
        (check_hypo)
        (game_finished)
    )
    
    (:durative-action motion
	:parameters (?obj - robot ?from ?to - waypoint)
	:duration ( = ?duration 5)
	:condition (and
		(at start (robot_at ?obj ?from))
		(at start (visited ?from))
		(at start (not(visited ?to))))
	:effect (and
	        (at end (robot_at ?obj ?to))
		(at end (visited ?to))
		(at start (not(robot_at ?obj ?from)))
    )
    
    (:durative-action move_arm
        :parameters (?obj - robot ?wp - waypoint)
	:duration ( = ?duration 3)
	:condition (and
		(at start (robot_at ?obj ?wp))
		(at start (gripper_on ?wp)))
	:effect (and
	        (at end (not(gripper_on ?wp))))
    )
    
    (:durative-action check_hypothesis
        :parameters (?obj - robot ?wp - waypoint)
	:duration ( = ?duration 1)
	:condition (and
		(at start (robot_at ?obj ?wp))
		(at start (not(check_hypo))))
	:effect (and
	        (at end (check_hypo)))
    )
    
    (:durative-action oracle
        :parameters (?obj - robot ?wp - waypoint)
	:duration ( = ?duration 1)
	:condition (and
		(at start (robot_at ?obj ?wp))
		(at start (check_hypo))
	:effect (and
	        (at end (game_finished)))
    )
   
)
