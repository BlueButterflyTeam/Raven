#include "Goal_FollowPath_Dodge.h"
#include "../Raven_Bot.h"
#include "../Raven_Game.h"

#include "Goal_TraverseEdge_Dodge.h"
#include "Goal_NegotiateDoor.h"
#include "../Common/misc/cgdi.h"

#include "Goal_TraverseEdge.h"

//------------------------------ ctor -----------------------------------------
//-----------------------------------------------------------------------------
Goal_FollowPath_Dodge::Goal_FollowPath_Dodge(Raven_Bot* pBot,
                std::list<PathEdge> path):Goal_Composite<Raven_Bot>(pBot, goal_follow_path_dodge),
                                                  m_Path(path)
{
}


//------------------------------ Activate -------------------------------------
//-----------------------------------------------------------------------------
void Goal_FollowPath_Dodge::Activate()
{
  m_iStatus = active;
  
  //get a reference to the next edge
  PathEdge edge = m_Path.front();

  //remove the edge from the path
  m_Path.pop_front(); 

  //some edges specify that the bot should use a specific behavior when
  //following them. This switch statement queries the edge behavior flag and
  //adds the appropriate goals/s to the subgoal list.
  switch(edge.Behavior())
  {
  case NavGraphEdge::normal:
    {
      //AddSubgoal(new Goal_TraverseEdge_Dodge(m_pOwner, edge, m_Path.empty()));
		if (m_pOwner->GetTargetSys()->isAggressed()){
			AddSubgoal(new Goal_TraverseEdge_Dodge(m_pOwner, edge, m_Path.empty()));
		}
		else{
			AddSubgoal(new Goal_TraverseEdge(m_pOwner, edge, m_Path.empty()));
		}
    }

    break;

  case NavGraphEdge::goes_through_door:
    {

      //also add a goal that is able to handle opening the door
      AddSubgoal(new Goal_NegotiateDoor(m_pOwner, edge, m_Path.empty()));
    }

    break;

  case NavGraphEdge::jump:
    {
      //add subgoal to jump along the edge
    }

    break;

  case NavGraphEdge::grapple:
    {
      //add subgoal to grapple along the edge
    }

    break;

  default:

    throw std::runtime_error("<Goal_FollowPath::Activate>: Unrecognized edge type");
  }
}


//-------------------------- Process ------------------------------------------
//-----------------------------------------------------------------------------
int Goal_FollowPath_Dodge::Process()
{
  //if status is inactive, call Activate()
  ActivateIfInactive();

  m_iStatus = ProcessSubgoals();

  //if there are no subgoals present check to see if the path still has edges.
  //remaining. If it does then call activate to grab the next edge.
  if (m_iStatus == completed && !m_Path.empty())
  {
    Activate(); 
  }

  return m_iStatus;
}
 
//---------------------------- Render -----------------------------------------
//-----------------------------------------------------------------------------
void Goal_FollowPath_Dodge::Render()
{ 
  //render all the path waypoints remaining on the path list
  std::list<PathEdge>::iterator it;
  for (it = m_Path.begin(); it != m_Path.end(); ++it)
  {  
    gdi->BlackPen();
    gdi->LineWithArrow(it->Source(), it->Destination(), 5);
    
    gdi->RedBrush();
    gdi->BlackPen();
    gdi->Circle(it->Destination(), 3);
  }

  //forward the request to the subgoals
  Goal_Composite<Raven_Bot>::Render();
}
  
/*
void addNormalSub(Raven_Bot* m_pOwner, PathEdge edge, bool endPath ){
	if (m_pOwner->GetTargetSys()->isAggressed()){
		AddSubgoal(new Goal_TraverseEdge_Dodge(m_pOwner, edge, endPath));
	}
	else{
		AddSubgoal(new Goal_TraverseEdge(m_pOwner, edge, endPath));
	}
}
*/

