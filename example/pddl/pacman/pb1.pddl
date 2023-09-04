
(define (problem win)
    (:domain pacman_bool)
    (:objects 
        f1 - fish
        f2 - c1
        f3 - c2
    )
    (:init 
        (food_at_playground f1)
        (food_at_playground f2)
        (food_at_playground f3)
        (enemy_at_home)
        (at_home)
    )

    (:goal (And
        (food_gained f1)
        (food_gained f2)
        (food_gained f3)
        (not (enemy_at_home))
    ))
)