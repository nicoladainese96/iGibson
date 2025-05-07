(define (problem cleaning_out_drawers_0)
    (:domain igibson)

    (:objects
     	bowl_1 bowl_2 - movable
    	cabinet_1 cabinet_2 - container
    	spoon_1 spoon_2 - movable
    	piece_of_cloth_1 - movable
    	sink_1 - object
    )
    
    (:init 
        (inside bowl_1 cabinet_1) 
        (inside bowl_2 cabinet_1) 
        (inside spoon_1 cabinet_2) 
        (inside spoon_2 cabinet_2) 
        (inside piece_of_cloth_1 cabinet_1) 

        (not (open cabinet_1))
        (not (open cabinet_2))
    )
    
    (:goal 
        (and 
            (ontop bowl_1 sink_1) 
        )
    )
)