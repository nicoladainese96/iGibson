(define (problem packing_adult_s_bags_0)
    (:domain igibson)

    (:objects
        backpack_1 - container
        floor_1 floor_2 - object
        bed_1 - bed.n.01
        makeup_1 makeup_2 - movable
        jewelry_1 jewelry_2 - movable
        toothbrush_1 - movable
        mouse_1 - movable
    )
    
    (:init 
        (ontop makeup1_1 bed_1) 
        (ontop makeup_2 bed_1) 
        (ontop toothbrush_1 bed_1) 
        (onfloor jewelry_1 floor_1) 
        (onfloor jewelry_2 floor_1) 
        (ontop mouse_1 bed_1) 
    )
    
    (:goal 
        (and 
            (inside jewelry_1 backpack_1)
            (inside jewelry_2 backpack_1)
            (inside makeup_1 backpack_1)
            (inside makeup_2 backpack_1)
            (inside toothbrush_1 backpack_1) 
            (inside mouse_1 backpack_1) 
        )
    )
)