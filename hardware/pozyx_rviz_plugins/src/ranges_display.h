#ifndef RANGES_DISPLAY_H
#define RANGES_DISPLAY_H

#ifndef Q_MOC_RUN
#include <rviz/message_filter_display.h>
#include <pozyx_msgs/Ranges.h>
#endif

namespace pozyx_rviz_plugins
{
class RangesVisual;

class RangesDisplay: public rviz::MessageFilterDisplay<pozyx_msgs::Ranges>
{
  Q_OBJECT
public:
  RangesDisplay();
  virtual ~RangesDisplay();

protected:
  virtual void onInitialize();

  virtual void reset();

private Q_SLOTS:
  void updateProperties();

private:
  void processMessage(const pozyx_msgs::Ranges::ConstPtr& msg);

  std::map<std::string, std::shared_ptr<RangesVisual>> frame_id_ranges_visual_map_;
};
}

#endif // WORLD_MODEL_DISPLAY_H
