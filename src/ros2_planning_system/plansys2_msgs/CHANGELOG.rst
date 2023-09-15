^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package plansys2_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.10 (2021-07-03)
-------------------
* Add ability to read in pddl problem files to plansys2 and a new AddProblem service to the plansys2_problem_expert.
  Adding problem_file node parameter to plansys2_problem_expert to load a single problem file at launch.
* ros2-plan-msg: Adding functions to plansys2_msgs/Knowledge message.
* ros2-plan-msg: Passing plan to executor to add further separation between plan creation and plan execution.
* pddl-tree-messages: Performing some minor cleanup.
* pddl-tree-messages: Merging master and resolving conflicts.
* Using custom behavior tree to enable action timeouts.
* pddl-tree-messages: Adding README to plansys2_msgs.
* pddl-tree-messages: Using ROS messages to define the PDDL construct trees.
* Plansys2_tests package
* Add GetOrderedSubGoals service to Executor, allowing executor clients to get the order in which sub-goals will be completed by the current plan.
* Update version
* Contributors: Alexander Xydes, Francisco Martín Rico, Josh Zapf

1.0.9 (2021-03-15)
------------------

1.0.8 (2021-03-12)
------------------
* Action execution refactoring
* Adding isGoalSatisfied function and unit tests to the problem expert api.
* Fixing typo in message name.
* Add support for numeric conditions and effects.
* Remove unused field
* Publish knownledge content
* Monitorization info
* fix minor typo
* Contributors: Alexander Xydes, Fabrice Larribe, Francisco Martin Rico, Josh Zapf

1.0.7 (2021-01-04)
------------------

1.0.6 (2020-12-29)
------------------

1.0.5 (2020-12-28)
------------------
* Migration to c++17
* Contributors: Francisco Martín Rico

1.0.4 (2020-12-24)
------------------

1.0.3 (2020-12-23)
------------------

1.0.2 (2020-12-23)
------------------
* Plan solvers as plugins
* Contributors: Fabrice Larribe, Francisco Martin Rico, f269858

1.0.1 (2020-07-19)
------------------


1.0.0 (2020-07-19)
------------------
* Foxy initial version
* Contributors: Francisco Martín Rico
0.0.8 (2020-07-18)
------------------

0.0.7 (2020-03-26)
------------------

0.0.6 (2020-03-23)
------------------

0.0.5 (2020-01-12)
------------------

0.0.4 (2020-01-09)
------------------
* Adding missing action dependencies
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martín Rico
0.0.3 (2020-01-09)
------------------

0.0.2 (2020-01-08)
------------------
* Packages.xml description
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Setting CI
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Change to lowercasegit
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Executor initial version
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* First version of planner complete
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Problem expert client and node
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Domain types and messages changed
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Predicate Tree and types changed
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* First version of domain expert
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martín Rico