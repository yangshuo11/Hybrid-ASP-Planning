#!/bin/bash

# automatically generate PDDL problem from KB snapshot (e.g. fetch knowledge from KB and create problem.pddl)
echo "Calling problem generator.";
rosservice call /rosplan_problem_interface/problem_generation_server;

# make plan (e.g. call popf to create solution)
echo "Calling planner interface.";
rosservice call /rosplan_planner_interface/planning_server;

# parse plan (parse console output and extract actions and params, e.g. create esterel graph)
echo "Calling plan parser.";
rosservice call /rosplan_parsing_interface/parse_plan;

# dispatch (execute) plan. (send actions one by one to their respective interface and wait for them to finish)
echo "Calling plan dispatcher.";
rosservice call /rosplan_plan_dispatcher/dispatch_plan;



echo "Finished!";