#ifndef GOAL_ENUMERATIONS_H
#define GOAL_ENUMERATIONS_H

#include <string>
#include "../Common/misc/TypeToString.h"

enum
{
  goal_think=0,
  goal_explore=1,
  goal_arrive_at_position=2,
  goal_seek_to_position=3,
  goal_follow_path=4,
  goal_follow_path_dodge=18,
  goal_traverse_edge=5,
  goal_traverse_edge_dodge=19,
  goal_move_to_position=6,
  goal_get_health=7,
  goal_get_shotgun=8,
  goal_get_rocket_launcher=9,
  goal_get_railgun=10,
  goal_wander=11,
  goal_negotiate_door=12,
  goal_attack_target=13,
  goal_hunt_target=14,
  goal_strafe=15,
  goal_adjust_range=16,
  goal_say_phrase=17
  
};

class GoalTypeToString : public TypeToString
{

  GoalTypeToString(){}

public:

  static GoalTypeToString* Instance();
  
  std::string Convert(int gt);
};

#endif
