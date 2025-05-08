(define (problem putting_away_toys_0)
    (:domain igibson)

    (:objects
        plaything_1 - movable
        carton_1 - container
        table_1 - object
    )
    
    (:init 
        (not (open carton_1))
    )
    
    (:goal 
        (and 
            (inside plaything_1 carton_1)
        )
    )
)