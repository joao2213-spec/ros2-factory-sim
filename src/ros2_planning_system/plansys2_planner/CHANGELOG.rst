^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package plansys2_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.10 (2021-07-03)
-------------------
* Revert wrong directory move
* Remove node parameter in client constructors
* ros2-plan-msg: Passing plan to executor to add further separation between plan creation and plan execution.
* pddl-tree-messages: Using explicit specifier for single parameter constructors in plansys2_core/Types.hpp.
* pddl-tree-messages: Replacing user access function calls with shorter versions where possible.
* pddl-tree-messages: Updating addInstance and removeInstance calls to use helper classes.
* pddl-tree-messages: Using ROS messages to define the PDDL construct trees.
* Update version
* Contributors: Francisco Martín Rico, Josh Zapf

1.0.9 (2021-03-15)
------------------

1.0.8 (2021-03-12)
------------------
* Update API for FutureReturnCode
* Add support for numeric conditions and effects.
* minor tipo
* Contributors: Fabrice Larribe, Francisco Martin Rico, Josh Zapf

1.0.7 (2021-01-04)
------------------
* Making explicit dependencies
* Contributors: Francisco Martín Rico
1.0.6 (2020-12-29)
------------------
* Disable boost functions
* Contributors: Francisco Martín Rico
1.0.5 (2020-12-28)
------------------
* Migration to c++17
* Contributors: Francisco Martín Rico

1.0.4 (2020-12-24)
------------------

1.0.3 (2020-12-23)
------------------
* Default plugin as library
* Contributors: Francisco Martín Rico
1.0.2 (2020-12-23)
------------------
* Plan solvers as plugins
* Contributors: Fabrice Larribe, Francisco Martin Rico, f269858

1.0.1 (2020-07-19)
------------------

1.0.0 (2020-07-19)
------------------
* Foxy initial version
* Contributors: Francisco Martin Rico

0.0.8 (2020-07-18)
------------------
* Boost:optional
* Contributors: Francisco Martin Rico

0.0.7 (2020-03-26)
------------------
* Fix warning in last cmake versions
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martín Rico
0.0.6 (2020-03-23)
------------------
* Run in separate namespaces. Monolothic node
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Set distro to eloquent, and activate disabled tests
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martin Rico

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
* Merge pull request `#16 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/16>`_ from IntelligentRoboticsLabs/pddl_parser_rename
  Rename pddl_parser
* Rename pddl_parser
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Merge pull request `#15 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/15>`_ from IntelligentRoboticsLabs/example_rename
  Rename example. Small bug in timeouts
* Rename example. Small bug in timeouts
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Packages.xml description
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Adding documentation
  Signed-off-by: Francisco Martin Rico <fmartin@gsyc.urjc.es>
* Setting CI
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Setting CI
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Setting CI
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Setting CI
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Setting CI
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Setting CI
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Setting CI
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* First version of planner complete
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martin Rico
