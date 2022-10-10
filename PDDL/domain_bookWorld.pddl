(define (domain bookWorld)

    (:requirements
        :equality
        :typing
        :strips
    )

    (:types
        robot
        book
        bin
        location
        subject
        size
    )


    (:predicates
        (Book_At ?a - book ?b - location)
        (Bin_At ?a - bin ?b - location)
        (Book_Subject ?a - book ?b - subject)
        (Book_Size ?a - book ?b - size)
        (Bin_Subject ?a - bin ?b - subject)
        (Bin_Size ?a - bin ?b - size)
        (Robot_At ?a - robot ?b - location)
        (Empty_Basket ?a - robot)
        (In_Basket ?b - book)
    )

    ; Pick up a book from the location.
    (:action pick
        :parameters (?book - book ?bot - robot ?locbk - location)
        :precondition (and
        (Empty_Basket ?bot)
        (Book_At ?book ?locbk)
        (not (In_Basket ?book))
		(Robot_At ?bot ?locbk)
        )
        :effect (and
		(not (Empty_Basket ?bot))
        (not (Book_At ?book ?locbk))
		(In_Basket ?book)
		(Robot_At ?bot ?locbk)

        )
    )

    ; Place the book on the robot into the bin.
    ; The robot must be at the drop-off location, must be holding the book, and
    ; the book subject and size must match that of the bin.
    (:action place
        :parameters (?book - book ?bot - robot ?bin - bin ?locbn - location ?sizebn - size ?subbn - subject)
        :precondition (and
            (Robot_At ?bot ?locbn)
            (not (Empty_Basket ?bot))

            (Book_Size ?book ?sizebn)
            (In_Basket ?book)
            (Book_Subject ?book ?subbn)

            (Bin_Size ?bin ?sizebn)
            (Bin_At ?bin ?locbn)
            (Bin_Subject ?bin ?subbn)
        )
        :effect (and
            (Robot_At ?bot ?locbn)
            (Empty_Basket ?bot)
            (not (In_Basket ?book))

            (Book_Size ?book ?sizebn)
            (Book_At ?book ?locbn)
            (Book_Subject ?book ?subbn)

            (Bin_Size ?bin ?sizebn)
            (Bin_At ?bin ?locbn)
            (Bin_Subject ?bin ?subbn)
        )
    )

    ; Move the robot from one location to another.
    (:action move
        :parameters (?bot - robot ?oldloc - location ?newloc - location)
        :precondition (and
        (not(Robot_At ?bot ?newloc))
		(Robot_At ?bot ?oldloc)
        )
        :effect (and
        (Robot_At ?bot ?newloc)
		(not(Robot_At ?bot ?oldloc))
        )
    )
)
