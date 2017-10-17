#include <despot/planner.h>
#include <laser_tag.h>
#include <laser_tag_world.h>

using namespace despot;

class MyPlanner: public Planner {
public:
  MyPlanner() {
  }

  DSPOMDP* InitializeModel(option::Option* options) {
    DSPOMDP* model = !options[E_PARAMS_FILE] ?
      new LaserTag() : new LaserTag(options[E_PARAMS_FILE].arg);
    return model;
  }

  World* InitializeWorld(std::string& world_type, DSPOMDP* model, option::Option* options)
  {
      //Create a custom world as defined and implemented by the user
      LaserTagWorld* world = new LaserTagWorld();
      //Establish connection with external system
      world->Connect();
      //Initialize the state of the external system
      world->Initialize();
      static_cast<LaserTag*>(model)->NoiseSigma(LaserTagWorld::noise_sigma_);
      //Inform despot the type of world
      world_type = "simulator";
      return world; 
  }

  void InitializeDefaultParameters() {
    Globals::config.pruning_constant = 0.01;
  }

  std::string ChooseSolver(){
	  return "DESPOT";
  }
};

int main(int argc, char* argv[]) {
  return MyPlanner().runPlanning(argc, argv);
}
