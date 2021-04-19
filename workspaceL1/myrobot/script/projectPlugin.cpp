#include <gazebo/gazebo.hh>

namespace gazebo
{
  class WorldPluginProject : public WorldPlugin
  {
    public: WorldPluginProject() : WorldPlugin()
            {
              printf("Welcome to Ben's World!\n");
            }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
            {
            }
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldPluginProject)
}
