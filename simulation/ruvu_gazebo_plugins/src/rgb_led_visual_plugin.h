#include <memory>
#include <gazebo/common/Plugin.hh>

namespace gazebo
{

class RGBLEDVisualPlugin : public VisualPlugin
{

public:

  void Load(rendering::VisualPtr visual, sdf::ElementPtr sdf);

private:

  rendering::VisualPtr visual_;

};

}
