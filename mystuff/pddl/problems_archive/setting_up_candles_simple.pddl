(define (problem setting_up_candles_0)
    (:domain igibson)

    (:objects
        candle_1 candle_2 candle_3 candle_4 candle_5 candle_6 - movable
        table_1 table_2 - object
        floor_1 floor_2 - object
        carton_1 carton_2 - container
    )
    
    (:init 
        (open carton_1)
        (inside candle_1 carton_1) 
        (inside candle_2 carton_1) 
        (inside candle_3 carton_1) 
        (inside candle_4 carton_1) 
        (inside candle_5 carton_1) 
        (inside candle_6 carton_1) 
    )
    
    (:goal 
        (and 
            (ontop candle_1 table_1)
        )
    )
)