(define (domain factory)
  (:requirements :strips :typing :durative-actions)
  (:types
    robot
    point
  )
  (:predicates
    (robot_at ?r - robot ?p - point)
  )
  (:durative-action move
    :parameters (?r - robot ?from - point ?to - point)
    :duration (= ?duration 5)
    :condition (and 
      (at start (robot_at ?r ?from))
    )
    :effect (and 
      (at start (not (robot_at ?r ?from)))
      (at end (robot_at ?r ?to))
    )
  )
)
