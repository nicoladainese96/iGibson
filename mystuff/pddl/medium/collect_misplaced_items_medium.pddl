(define (problem collect_misplaced_items_0)
    (:domain igibson)

    (:objects
        necklace_1 - movable
        notebook_1 - movable
        table_1 table_2 - object
        cabinet_1 - container
        sofa_1 - object
        floor_1 floor_2 - object
    )
    
    (:init 
        (inside necklace_1 cabinet_1) 
    )
    
    (:goal 
        (and 
            (ontop necklace_1 table_2) 
            (ontop notebook_1 table_2) 
        )
    )
)