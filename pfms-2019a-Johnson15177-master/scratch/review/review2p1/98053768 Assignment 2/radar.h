#ifndef RADAR_H
#define RADAR_H

#include "ranger.h"

/*!
 *  \author    Samuel Funk
 *  \version   1.0
 *  \date      2019-04-23
 *  \bug       none reported as of 2019-04-23
 * \details
 * Facilitates the creation of a mock radar of model RAD-001
 * and the setting of specific angular resolution, offset and
 * field of view parameters.\n\n
 * The radar's angular resolution and field of view are preset to 20
 * and 60 degrees, respectively. The offset value is limited to values
 * in the range -120 to +120 degrees. It is preset to 0 by default.\n\n

 * The radar generates a list of distance readings. The first reading corresponds to an
 * an angle of -(field of view)/2 deg from the offset, and encompasses a range from
 * -(field of view)/2 to (-(field of view/2) + (angular resolution)). The final angle
 * of the first reading is the starting angle of the second reading, and so on, until
 * the final reading end at reaches the maximum of +(field of view)/2 deg from the offset.\n\n
 * For example, if the offset is set to 0 deg, then the readings will correspond
 * to angle ranges in the following order:\n
 * -30 to -10, -10 to 10, 10 to 30.\n\n
 * If the offset is set to 20 deg, then the readings will correspond
 * to angle rangers in the following order:\n
 * -10 to 10, 10 to 30, 30 to 50.\n\n
 * If the offset is set to -40 deg, then the readings will correspond
 * to angle ranges in the following order:\n
 * -70 to -50, -50 to -30, -30 to -10.\n\n

 * USAGE:
 * 1. Instantiate an object of the Radar class.
 * 2. Optionally change the offset using setOffset()\n
 */

class Radar: public Ranger
{
public:
  //Default constructor to set all sensor attributes to a default value
  Radar();

  //Redefining some virtual functions

  /*! As the angular resolution is preset to 20 degrees,
   * this function only checks if the inputted value for
   * angular resolution is 20 degrees.
   @param[in] ang_res Desired angular resolution value.
   *\return True if ang_res == 20, false otherwise.
   */
  bool setAngularResolution(unsigned int);

  /*! Sets the desired offset value for the radar. If the offset is outside the
   * permissible range, the offset is instead set to the default value of 0 and
   * false is returned.
   @param[in] offset Desired offset value.
   *\return True if -120 < offset < 120, false otherwise.
   */
  bool setOffset(int);

  /*! As the field of view is preset to 60 degrees,
   * this function only checks if the inputted value for
   * field of view is 60 degrees.
   @param[in] fov Desired field of view value.
   *\return True if fov == 60, false otherwise.
   */
  bool setFieldOfView(unsigned int);

};

// PUT THIS IN DOXYGEN DOCUMENTATION:
/* The radar generates a list of readings. The first reading corresponds to the
 * an angle of -30 deg from the offset, and encompasses a range set by the
 * angular resolution. The final angle of the first reading is the starting angle
 * of the second reading, and so on, until the final reading end at reaches the
 * maximum of +30 deg from the offset.
 * For example, if the offset is set to 0 deg, then the readings will correspond
 * to angles in the following order:
 * -30 to -10, -10 to 10, 10 to 30.
 * If the offset is set to 20 deg, then the readings will correspond
 * to angles in the following order:
 * -10 to 10, 10 to 30, 30 to 50.
 * If the offset is set to -40 deg, then the readings will correspond
 * to angles in the following order:
 * -70 to -50, -50 to -30, -30 to -10.
 * (Note the angular resolution is fixed at 20 deg and the field of view is
 *  fised at 180 deg)
 */

#endif // RADAR_H
