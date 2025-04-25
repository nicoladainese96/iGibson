(define (problem cleaning_out_drawers-0)
    (:domain igibson)

    (:objects
     	bowl_n_01_1 bowl_n_01_2 - object
    	cabinet_n_01_1 cabinet_n_01_2 - container
    	tablespoon_n_02_1 tablespoon_n_02_2 - object
    	dinner_napkin_n_01_1 - object
    	sink_n_01_1 - container
    	agent_n_01_1 - agent
    )
    
    (:init 
        (inside bowl_n_01_1 cabinet_n_01_1) 
        (inside bowl_n_01_2 cabinet_n_01_1) 
        (inside tablespoon_n_02_1 cabinet_n_01_2) 
        (inside tablespoon_n_02_2 cabinet_n_01_2) 
        (inside dinner_napkin_n_01_1 cabinet_n_01_1) 

        (movable dinner_napkin_n_01_1)
        (movable bowl_n_01_1)
        (movable bowl_n_01_2)
        (movable tablespoon_n_02_1)
        (movable tablespoon_n_02_2)
        
        (openable cabinet_n_01_1)
        (openable cabinet_n_01_2)
    )
    
    (:goal 
        (and 
            (ontop dinner_napkin_n_01_1 sink_n_01_1) 
            (ontop bowl_n_01_1 sink_n_01_1) 
            (ontop bowl_n_01_2 sink_n_01_1) 
            (ontop tablespoon_n_02_1 sink_n_01_1) 
            (ontop tablespoon_n_02_2 sink_n_01_1)
        )
    )
)