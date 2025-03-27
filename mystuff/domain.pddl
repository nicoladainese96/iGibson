(define (domain igibson)
    (:requirements :strips :typing :negative-preconditions :conditional-effects :equality :disjunctive-preconditions)

    (:types
        agent receptacle - object
        container - receptacle ;; A container is a type of receptacle that can be opened and closed
    )

    (:predicates

        ;; Agent predicates
        (reachable ?o - object ?a - agent)
        (holding ?a - agent ?o - object)

        ;; Object attributes
        (movable ?o - object)
        (openable ?o - object) ;; equivalent to is-container
        (open ?c - container)

        ;; Object relations
        (ontop ?o1 - object ?o2 - object)
        (inside ?o - object ?r - object)

        ;; Specific object attributes
        (slicer ?o - object) ;; (e.g. knife)
        (sliced ?o - object) ;; (e.g. sliced tomato)
    )

    (:action grasp
        :parameters (?a - agent ?o - object)
        :precondition (and
            (reachable ?o ?a)
            (movable ?o)
            (forall
                (?x - object)
                (not (holding ?a ?x))) ;; Agent must not be holding anything
            (forall
                (?x - object)
                (not (ontop ?x ?o))) ;; Can't grasp an objectect that has something on top of it
        )
        :effect (and
            (holding ?a ?o)
            (not (reachable ?o ?a))
            (forall
                (?y - object)
                (not (ontop ?o ?y))) ;; If grasped objectect is on top of something, it is no longer on top of it
            (forall
                (?r - receptacle)
                (when
                    (inside ?o ?r)
                    (not (inside ?o ?r)))) ;; If o was in a receptacle, it's not anymore
        )
    )

    (:action place-on
        :parameters (?a - agent ?o1 - object ?o2 - object)
        :precondition (and
            (holding ?a ?o1)
            (reachable ?o2 ?a)
        )
        :effect (and
            (ontop ?o1 ?o2)
            (not (holding ?a ?o1))
        )
    )

    (:action place-inside
        :parameters (?a - agent ?o - object ?r - receptacle)
        :precondition (and
            (holding ?a ?o)
            (reachable ?r ?a)
        )
        :effect (and
            (inside ?o ?r)
            (not (holding ?a ?o))
        )
    )

    (:action open-container
        :parameters (?a - agent ?c - container)
        :precondition (and
            (openable ?c)
            (reachable ?c ?a)
            (not (open ?c))
        )
        :effect (and
            (open ?c)
            (forall
                (?o - object)
                (when
                    (inside ?o ?c)
                    (reachable ?o ?a))) ;; All objects inside the container are reachable
        )
    )

    (:action close-container
        :parameters (?a - agent ?c - container)
        :precondition (and
            (openable ?c)
            (reachable ?c ?a)
            (open ?c)
        )
        :effect (and
            (not (open ?c))
            (forall
                (?o - object)
                (when
                    (inside ?o ?c)
                    (not (reachable ?o ?a)))) ;; All objects inside the container are unreachable
        )
    )

    (:action navigate-to
        :parameters (?a - agent ?o - object)
        :precondition (not (reachable ?o ?a))
        :effect (and
            ;; Make all other objects unreachable.
            (forall
                (?x - object)
                (when
                    (not (= ?x ?o))
                    (not (reachable ?x ?a))))
            ;; If ?o is inside a closed container, mark that container reachable instead
            (when
                (exists
                    (?c - container)
                    (and (inside ?o ?c) (not (open ?c))))
                (forall
                    (?c - container)
                    (when
                        (and (inside ?o ?c) (not (open ?c)))
                        (and (reachable ?c ?a) (not (reachable ?o ?a))))))
            ;; Otherwise, mark ?o reachable.
            (when
                (forall
                    (?c - container)
                    (or (not (inside ?o ?c)) (open ?c)))
                (reachable ?o ?a))
            ;; If ?o is not openable and has something inside (a receptacle), mark those objects as reachable.
            (when
                (and
                    (not (openable ?o))
                    (exists
                        (?x - object)
                        (inside ?x ?o)))
                (forall
                    (?x - object)
                    (when
                        (inside ?x ?o)
                        (reachable ?x ?a))))
        )
    )

    (:action slice
        :parameters (?a - agent ?o - object)
        :precondition (and
            (exists (?s - object) 
                (and 
                    (holding ?a ?s)
                    (slicer ?s)))
            (reachable ?o ?a)
            (not (sliced ?o))
        )
        :effect (and 
            (sliced ?o)
        )
    )
    
)