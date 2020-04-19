/*! @file
 *
 *  @brief A library of simple types used by the simulator class.
 *
 *  @author arosspope
 *  @date 24-08-2018
*/
#ifndef TYPES_H
#define TYPES_H

#include <vector>
#include <mutex>
#include "timer.h"

struct GlobalOrd {
  double x;   /*!< x coordinate within global map (m) */
  double y;   /*!< y coordinate within global map (m) */

  /*!< Implementing the operator '==' for this struct */
  bool operator== (const GlobalOrd &o1){
    return (this->x == o1.x && this->y == o1.y);
  }
};

struct Pose {
  GlobalOrd position; /*!< Global position (metres) */
  double orientation; /*!< Orientation (radians) */
};

struct RangeStamped { /*!< Contains a timestamped range (distance reading) */
  double range;       /*!< The range (distance reading) in metres */
  long timestamp;     /*!< Timestamp (milliseconds) */
};

struct Aircraft {
  Pose pose;                    /*!< Global position and orientation within the airspace */
  std::vector<GlobalOrd> trail; /*!< To display where the aircraft has been */
  double linear_velocity;       /*!< Linear velocity (metres/second) */
  double angular_velocity;      /*!< Angular velocity (radians/second). (+) Counter clockwise, (-) Clockwise. */
  Timer timer;                  /*!< Used to keep track of elapsed time. */
};

struct AircraftContainer { /*!< A thread safe container for the Aircraft type */
  Aircraft a;
  std::mutex access;
};

#endif // TYPES_H
