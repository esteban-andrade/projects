#ifndef LASER_H
#define LASER_H

#include "ranger.h"

/*!
 *  \author    Samuel Funk
 *  \version   1.0
 *  \date      2019-04-23
 *  \bug       none reported as of 2019-04-23
 * \details
 * Facilitates the creation of a mock laser of model UTM-XXL
 * and the setting of specific angular resolution, offset and
 * field of view parameters\n\n
 * The laser's offset and field of view are preset to 0 and 180
 * degrees, respectively, and the angular resolution value is limited
 * to values or either 10 or 30 degrees. By default, it is preset to 30.\n\n

 * The laser generates a list of readings. The first reading corresponds to the
 * an angle of -(field of view)/2 deg from the offset, and each reading's angle increases
 * by the value of the angular resolution, until it reaches the maximum of (field of view)/2.\n\n
 * For example, if the angular resolution is set to 30, then the readings will
 * correspond to angles in the following order:\n
 * -90, -60, -30, 0, 30, 60, 90.\n\n
 * If the angular resolution is set to 10, the readings will correspond to angles
 * in this order:\n
 * -90, -80, -70, -60, -50, -40, -30, -20, -10, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90.\n\n

 * USAGE:
 * 1. Instantiate an object of the Laser class.
 * 2. Optionally change the angular resolution using setAngularResolution().\n

 */

class Laser: public Ranger
{
public:
  //Default constructor - should set all sensor attributes to a default value
  Laser();

  //Redefining some virtual functions


  /*! Sets the desired angular resolution value for the radar. If a value other
   * than 10 or 30 degrees is input, the function returns false and sets the
   * angular resolution to the default value of 30.
   @param[in] ang_res Desired angular resolution value.
   *\return True if ang_res == 10 or ang_res == 30, false otherwise.
   */
  bool setAngularResolution(unsigned int);

  /*! As the offset is preset to 0 degrees, this function
   * only checks if the inputted value for offset is 0 degrees.
   @param[in] offset Desired offset value.
   *\return True if offset == 0, false otherwise.
   */
  bool setOffset(int);

  /*! As the field of view is preset to 180 degrees,
   * this function only checks if the inputted value for
   * field of view is 180 degrees.
   @param[in] fov Desired field of view value.
   *\return True if fov == 180, false otherwise.
   */
  bool setFieldOfView(unsigned int);
};

// PUT THIS IN DOXYGEN DOCUMENTATION:
/* The laser generates a list of readings. The first reading corresponds to the
 * an angle of -90 deg (from the offset), and each reading's angle increases
 * by the value of the angular resolution, until it reaches the maximum of 90 deg.
 * For example, if the angular resolution is set to 30, then the readings will
 * correspond to angles in the following order:
 * -90, -60, -30, 0, 30, 60, 90.
 * If the angular resolution is set to 10, the readings will correspond to angles
 * in this order:
 * -90, -80, -70, -60, -50, -40, -30, -20, -10, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90.
 * (Note the offset is fixed at 0 deg and the field of view is fixed at 180 deg)
 */

#endif // LASER_H
