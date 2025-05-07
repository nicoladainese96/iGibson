(define (problem sorting_books_0)
    (:domain igibson)

    (:objects
     	hardback_1 hardback_2 - movable
    	table_1 - object
    	floor_1 - object
    	shelf_1 - object
    	book_1 book_2 - movable
    )
    
    (:init 
        (ontop hardback_1 table_1) 
        (ontop book_2 table_1) 
    )
    
    (:goal 
        (and 
            (ontop hardback_1 shelf_1)
        )
        
    )
)