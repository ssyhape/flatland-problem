
(define (problem win)
    (:domain pacman_bool)
    (:objects 
        f1
        f2
        f3
        f4
        f5
        f6   
    )
    (:init 
        (food_at_playground f1)
        (food_at_playground f2)
        (food_at_playground f3)
        (food_at_playground f4)
        (food_at_playground f5)
        (food_at_playground f6)
        (enemy_at_home)
        (at_enemy_land)
    )

    (:goal (And
        (food_gained f1)
        (food_gained f2)
        (food_gained f3)
        (food_gained f4)
        (food_gained f5)
        (food_gained f6)
        (not (enemy_alive))

    ))
)