#pragma once

#include <algorithm>
#include <map>
#include <stdint.h>

#define blueCone 0
#define yellowCone 1
#define orangeCone 2
#define orangeBigCone 3

enum class ColorId : uint8_t {
  blue_cone,
  yellow_cone,
  orange_cone,
  orange_big_cone,
  end,
};

class ColorLogic {
public:
  explicit ColorLogic() {
    /* Initialize color counter */
    for (uint8_t i = 0; i != static_cast<uint8_t>(ColorId::end); ++i)
      cone_info[static_cast<ColorId>(i)] = 0;
  }

  /* Increments the color counter */
  void setColor(uint8_t color_id) {

    ColorId color = static_cast<ColorId>(color_id);

    if (cone_info.find(color) == cone_info.end()) {
      return;
    }

    ++cone_info.at(color);
  }

  /* Return the color of the most seen cone */
  ColorId getConeColor() {
    auto output_cone =
        std::max_element(cone_info.begin(), cone_info.end(),
                         [](const std::pair<ColorId, uint32_t> &p1,
                            const std::pair<ColorId, uint32_t> &p2) {
                           return p1.second < p2.second;
                         });

    return output_cone->first;
  }

  /* Return the color of the most seen cone + the times it has been seen */
  std::pair<ColorId, uint32_t> getConeColorAndCount() {
    auto output_cone =
        std::max_element(cone_info.begin(), cone_info.end(),
                         [](const std::pair<ColorId, uint32_t> &p1,
                            const std::pair<ColorId, uint32_t> &p2) {
                           return p1.second < p2.second;
                         });
    return *output_cone;
  }

private:
  std::map<ColorId, uint32_t> cone_info;
};