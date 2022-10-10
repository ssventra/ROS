(define (domain cafeWorld)

    (:requirements
        :equality
        :typing
        :strips
    )

    (:types
        robot
        food
        table
        location
        food_type
        size
    )


    (:predicates
        (Food_At ?a - food ?b - location)
        (Table_At ?a - table ?b - location)
        (Food_Type ?a - food ?b - food_type)
        (Robot_At ?a - robot ?b - location)
        (Empty_Basket ?a - robot)
        (In_Basket ?b - food)
        (Ordered ?table - table ?food_type - food_type)
        (Portion_Size ?food - food ?size - size)
        (Ordered_Portion ?table - table ?size - size)
    )

    ; Pick up a food from the location.
    (:action pick
        :parameters (?food - food ?bot - robot ?locbk - location)
        :precondition (and
        (Robot_At ?bot ?locbk)
		(Food_At ?food ?locbk)
        (Empty_Basket ?bot)
        )
        :effect (and
        (Robot_At ?bot ?locbk)
        (In_Basket ?food)
        (not(Food_At ?food ?locbk))
        (not (Empty_basket ?bot))
        )
    )

    ; Place the food on the robot on the table.
    ; The robot must be at the drop-off location, must be holding the food, and
    ; the food type and size must match that of the table ordering the food.
    (:action place
        :parameters (?food - food ?bot - robot ?table - table ?locbn - location ?sizebn - size ?type - food_type)
        :precondition (and
            (Robot_At ?bot ?locbn)
            (Portion_Size ?food ?sizebn)
            (In_Basket ?food)
            (Food_Type ?food ?type)

            (Ordered_Portion ?table ?sizebn)
            (Table_At ?table ?locbn)
            (Ordered ?table ?type)
        )
        :effect (and
            (Robot_At ?bot ?locbn)
            (Empty_Basket ?bot)
            (not (In_Basket ?food))

            (Portion_Size ?food ?sizebn)
            (Food_At ?food ?locbn)

        )
    )

    ; Move the robot from one location to another.
    (:action move
        :parameters (?bot - robot ?oldloc - location ?newloc - location)
        :precondition (and
		(Robot_At ?bot ?oldloc)

		(not(Robot_At ?bot ?newloc))
        )
        :effect (and
		(not(Robot_At ?bot ?oldloc))

		(Robot_At ?bot ?newloc)
        )
    )
)
