(define (problem test_problem)
    (:domain igibson)
    (:objects
        agent1 - agent
        tomato1 lettuce1 fork1 knife1 table1 table2 - object
        plate1 - receptacle 
        cabinet1 fridge1 - container
    )

    (:init
        (ontop plate1 table1)
        (ontop fork1 table1)
        (inside knife1 cabinet1)

        (movable plate1)
        (movable fork1)
        (movable knife1)

        (openable cabinet1)
        (openable fridge1)

        (inside tomato1 fridge1)
        (inside lettuce1 fridge1)
        
        (slicer knife1)
    )

    (:goal
        (and
            (ontop plate1 table2)
            (inside fork1 plate1)
            (not (inside knife1 plate1))
            (ontop knife1 table1)
            (not (open cabinet1))

            (sliced tomato1)
            (sliced lettuce1)
        )
    )
)