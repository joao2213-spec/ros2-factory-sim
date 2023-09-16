(define (domain factory)
  (:requirements :strips :typing)
  (:types
    robot
    point
  )
  (:predicates
    (robot_at ?r - robot ?p - point)
  )
  (:action move
    :parameters (?r - robot ?from - point ?to - point)
    :precondition (robot_at ?r ?from)
    :effect (and 
              (not (robot_at ?r ?from))
              (robot_at ?r ?to)
            )
  )
)
