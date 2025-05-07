(define (problem unpacking_suitcase_0)
    (:domain igibson)

    (:objects
    	floor_1 - object
        carton_1 - container
        perfume_1 - movable
        toothbrush_1 - movable
        notebook_1 - movable
    	sofa_1 - object
    )
    
    (:init 
        (inside perfume_1 carton_1) 
        (inside toothbrush_1 carton_1) 
        (inside notebook_1 carton_1) 
    )
    
    (:goal 
        (and 
            (ontop perfume_1 sofa_1) 
            (ontop toothbrush_1 sofa_1) 
            (ontop notebook_1 sofa_1)
        )
    )
)