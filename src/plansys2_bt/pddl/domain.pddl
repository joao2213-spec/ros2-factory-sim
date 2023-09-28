(define (domain factory)
  (:requirements :strips :typing :durative-actions)
  (:types
    robot
    point
    package
  )
  (:predicates
    (robot_at ?r - robot ?p - point)
    (package_at ?r - package ?p - point)
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
  (:durative-action transport
    :parameters (?r - robot ?p - package ?z1 ?z2 - zone)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?z1))
        (at start(package_at ?p ?z1))
    )
    :effect (and
        (at start(not(robot_at ?r ?z1)))
        (at end(robot_at ?r ?z2))
        (at start(not(package_at ?p ?z1)))
        (at end(package_at ?p ?z2))
    )
  )
)
