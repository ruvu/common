#include "rgb_led_visual_plugin.h"
#include "util.h"

#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/Visual.hh>

namespace gazebo
{

void RGBLEDVisualPlugin::Load(rendering::VisualPtr visual, sdf::ElementPtr sdf)
{
  if (!visual || !sdf)
  {
    gzerr << "No visual or SDF element specified. Plugin won't load." <<
        std::endl;
    return;
  }
  visual_ = visual;

  std::cout << "\n\n\n\nloading\n\n\n\n" << std::endl;

  unsigned int child_count = visual_->GetParent()->GetChildCount();
  std::cout << child_count << std::endl;
  for (unsigned int i = 0; i < child_count; ++i)
  {
    std::cout << visual_->GetParent()->GetChild(i)->GetName() << std::endl;
    common::Color color(i * 0.1, i * 0.1, 0, 1.0);
    visual_->GetParent()->GetChild(i)->SetDiffuse(color);
    visual_->GetParent()->GetChild(i)->SetAmbient(color);
    visual_->GetParent()->GetChild(i)->SetTransparency(1-color.a);
  }
}

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(RGBLEDVisualPlugin)

}
