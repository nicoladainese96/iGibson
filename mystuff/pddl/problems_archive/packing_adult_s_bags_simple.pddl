(define (problem packing_adult_s_bags_0)
    (:domain igibson)

    (:objects
        backpack_1 - container
        floor_1 floor_2 - object
        bed_1 - object
        makeup_1 makeup_2 - movable
        jewelry_1 jewelry_2 - movable
        toothbrush_1 - movable
        mouse_1 - movable
    )
    
    (:init 
        (ontop makeup_1 bed_1) 
        (ontop makeup_2 bed_1) 
        (ontop toothbrush_1 bed_1) 
        (ontop jewelry_1 floor_1) 
        (ontop jewelry_2 floor_1) 
        (ontop mouse_1 bed_1) 
    )
    
    (:goal 
        (and 
            (inside toothbrush_1 backpack_1) 
        )
    )
)