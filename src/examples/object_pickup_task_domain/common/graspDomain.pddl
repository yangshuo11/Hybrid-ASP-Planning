(define (domain graspDomain)
    (:requirements :strips :fluents :durative-actions :timed-initial-literals :typing :conditional-effects :negative-preconditions :duration-inequalities :equality)
    (:types glass table waypoint)

    (:predicates
        (in_hand ?g - glass)
        (emptyhand)
        (on_table ?g - glass ?t - table)
        (robot_at ?wp - waypoint)
        (near ?wp - waypoint ?t - table)
    )

    (:functions
    )

    (:durative-action pick
        :parameters (?g - glass ?t - table ?wp - waypoint)
        :duration (= ?duration 60)
        :condition (and
            (over all (robot_at ?wp))
            (over all (near ?wp ?t))
            (at start (on_table ?g ?t))
            (at start (emptyhand))
            )
        :effect (and
            (at end (in_hand ?g))
            (at end (not (emptyhand)))
            (at end (not (on_table ?g ?t)))
        )
    )

    (:durative-action place
        :parameters (?g - glass ?t - table ?wp - waypoint)
        :duration (= ?duration 60)
        :condition (and
            (over all (robot_at ?wp))
            (over all (near ?wp ?t))
            (at start (in_hand ?g))
            )
        :effect (and
            (at end (not (in_hand ?g)))
            (at end (emptyhand))
            (at end (on_table ?g ?t))
        )
    )

    (:durative-action goto_waypoint
        :parameters (?from ?to - waypoint)
        :duration (= ?duration 20)
        :condition (and
            (at start (robot_at ?from)))
        :effect (and
            (at start (not (robot_at ?from)))
            (at end (robot_at ?to))
            )
    )
)