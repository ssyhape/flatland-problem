;Header and description

(define (domain pacman_bool)

;remove requirements that are not needed
(:requirements :strips :typing)

; un-comment following line if constants are needed
;(:constants )
(:types 
    food - object
    c1 - capsule
    c2 - capsule
    fish - food
    )

(:predicates 
    (enemy_around)
    (enemy_at_home) 
    (at_home) 
    (at_enemy_land)
    (enemy_alive) 
    (food_backpack ?x )  
    (food_at_playground ?x)
    (food_gained ?x)
)

;define actions here

(:action go_to_enemy_playground
    :parameters ()
    :precondition (and (at_home) )
    :effect (and (not (at_home )) (at_enemy_land ))
)

(:action go_to_enemy
    :parameters ()
    :precondition (and 
        (at_home)
        (enemy_at_home)
    )
    :effect (and 
        (enemy_around)
    )
)

(:action eat_enemy
    :parameters ()
    :precondition (and (at_home ) (enemy_around) (enemy_alive))
    :effect (and 
    (not (enemy_around))
    (not (enemy_at_home))
    (not (enemy_alive))

    )
)

(:action eat_food_1
    :parameters (?x )
    :precondition (and (not (enemy_around)) (not (enemy_alive)) (at_enemy_land ) (food_at_playground ?x ))
    :effect (and 
        (not (food_at_playground ?x))
        (food_backpack ?x)
        (enemy_alive)
    )
)

(:action eat_food_2
    :parameters (?x )
    :precondition (and (not (enemy_around))  (enemy_alive) (at_enemy_land ) (food_at_playground ?x ))
    :effect (and 
        (not (food_at_playground ?x))
        (food_backpack ?x)
        (enemy_at_home)
    )
)
(:action go_home
    :parameters ()
    :precondition (and (at_enemy_land ) )
    :effect (and 
        (not (at_enemy_land ))
        (at_home )
    )
)

(:action unpack_food
    :parameters (?x)
    :precondition (and (at_home) (food_backpack ?x))
    :effect (and 

        (not (food_backpack ?x))
        (food_gained ?x)

    
    )
    
    
)


)
