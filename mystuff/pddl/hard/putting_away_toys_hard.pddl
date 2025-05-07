(define (problem putting_away_toys_0)
    (:domain igibson)

    (:objects
        plaything_1 plaything_2 plaything_3 plaything_4 plaything_5 plaything_6 plaything_7 plaything_8 - movable
        floor_1 floor_2 - object
        carton_1 carton_2 - container
        table_1 - object
    )
    
    (:init 
        (ontop plaything_1 floor_1) 
        (ontop plaything_2 floor_1) 
        (not (open carton_1))
        (not (open carton_2))
    )
    
    (:goal 
        (and 
            (inside plaything_1 carton_1)
            (inside plaything_2 carton_2)
        )
    )
)