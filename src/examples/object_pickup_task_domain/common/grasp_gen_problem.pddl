(define (problem task)
(:domain graspdomain)
(:objects
    glass_cup - glass
    table0 table1 - table
    wp0 wp1 - waypoint
)
(:init

    (emptyhand)

    (on_table glass_cup table0)

    (robot_at wp0)

    (near wp0 table0)
    (near wp1 table1)

)
(:goal (and
    (on_table glass_cup table1)
))
)
