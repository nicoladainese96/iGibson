(define (problem putting_away_toys_0)
    (:domain igibson)

    (:objects
        plaything_1 plaything_2 - movable
        carton_1 carton_2 - container
        table_1 - object
    )
    
    (:init 
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