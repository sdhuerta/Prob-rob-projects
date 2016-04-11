#ifndef SDSMT_TRANSITIONMODEL
#define SDSMT_TRANSITIONMODEL

#include <vector>
#include <map.h>

struct ActionDef{
  double linear_vel;
  double angular_vel;
};

using namespace std;

class TransitionModel{
 private:
  
  int nActions;
  int nAngles;
  ActionDef *actions;
  vector<MapCell> **transitions;
  void deallocate();
  void allocate();

 public:
  TransitionModel():nActions(0),nAngles(0),actions(NULL),transitions(NULL){};
  TransitionModel(int num_actions, int num_angles);
  ~TransitionModel();

  int numActions(){return nActions;}
  int numAngles(){return nAngles;}
  
  vector<MapCell> &getTransitions(const MapCoord &s, const int action);
  void addTransition(MapCoord s, int a, MapCoord sp, double prob);

  ActionDef &getAction(int a){return actions[a];}
  void setAction(int a,ActionDef &d){actions[a]=d;}

  // getReward may change sp if there is a collision.
  double getReward(const MapCoord &s, const int a, MapCoord &sp,
		   const MapStruct *m, const MapCell *goals, const int nGoals);

  void read(char *filename);
  void write(char *filename);
};  


#endif
