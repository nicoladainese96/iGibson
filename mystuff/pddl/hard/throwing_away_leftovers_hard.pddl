(define (problem throwing_away_leftovers_0)
    (:domain igibson)

    (:objects
     	plate_1 plate_2 plate_3 - object
        hamburger_1 hamburger_2 hamburger_3 - movable
    	countertop_1 - object
    	ashcan_1 - container
    )
    
    (:init 
        (ontop plate_1 countertop_1) 
        (ontop hamburger_1 plate_1) 

        (ontop plate_2 countertop_1) 
        (ontop hamburger_3 plate_2) 

        (ontop plate_3 countertop_1) 
        (ontop hamburger_2 plate_3) 

    )
    
    (:goal 
        (and 
            (inside hamburger_1 ashcan_1)
            (inside hamburger_2 ashcan_1)
            (inside hamburger_3 ashcan_1)
        )
    )
)
