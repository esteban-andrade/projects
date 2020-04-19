#ifndef RANGER_H
#define RANGER_H

#include <chrono>
#include <random>
#include <string>
#include "rangerinterface.h"

/*!
 *  \author    Samuel Funk
 *  \version   1.0
 *  \date      2019-04-23
 *  \bug       none reported as of 2019-04-23
 *  \details
 *  Facilitates the creation of mock ranged sensors, the setting of their attributes and the generation of dummy data readings.\n
 * NOTE: an object of Ranger class cannot be created directly. Only objects of the subclasses Laser and Radar can be created.
 */

class Ranger: public RangerInterface
{
public:
  // Generates raw data for sensor
  /*! Generates raw data from the sensor.\n
   * Each data value is a random value with the a mean and standard deviation set in
   * setMean() and setStdDev().
   *
   * \return Vector of data values.
   * \sa setMean(), setStdDev()
   */
  std::vector<double> generateData();

  // Getters for obtaining internal private variables

  /*! \return The model name of the sensor.
   * \sa setModel()
   */
  std::string getModel(void);

  /*! \return The current angular resolution of the sensor.
   * \sa setAngularResolution()
   */
  unsigned int getAngularResolution(void);

  /*! \return The current offset of the sensor.
   * \sa setOffset()
   */
  int getOffset(void);

  /*! \return The current field of view of the sensor.
   * \sa setFieldOfView()
   */
  unsigned int getFieldOfView(void);

  /*! \return The maximum range of the sensor.
   */
  double getMaxRange(void);

  /*! \return The minimum range of the sensor.
   */
  double getMinRange(void);

  /*! \return The current mean value used to generate data.
   * \sa setMean()
   */
  double getMean(void);

  /*! \return The current standard deviation value used to generate data.
   * \sa setStdDev()
   */
  double getStdDev(void);

  /*!
   * \return Current sample number.
   * \sa setSampleNum()
   */
  int getSampleNum(void);

  // Setters for setting internal private variables

  /*! Sets the value for angular resolution.\n
   @param[in] ang_res Desired angular resolution value.\n
   * \return True.
   * \sa getAngularResolution()
   */
  virtual bool setAngularResolution(unsigned int);

  /*! Sets the value for sensor offset.\n
   @param[in] offset Desired offset value.\n
   * \return True.
   * \sa getOffset()
   */
  virtual bool setOffset(int);


  /*! Sets the value for field of view.\n
   @param[in] fov Desired field of view value.\n
   * \return True.
   * \sa getFieldOfView()
   */
  virtual bool setFieldOfView(unsigned int);

  /*! Sets the value for the mean value for data generation.
   @param[in] mean Desired mean value.\n
   * \return True if the mean is a valid distance (greater than zero), false otherwise.
   * \sa getMean()
   */
  bool setMean(double);

  /*! Sets the value for the standard deviation value for data generation.
   @param[in] sd Desired standard deviation value.\n
   * \return True if the standard deviation  is greater than zero, false otherwise.
   * \sa getStdDev()
   */
  bool setStdDev(double);

  /*! Sets the current sample number.\n
   @param[in] sample_num Desired sample number value.\n
   * \return True if the set number is valid (i.e greater than zero); false otherwise.
   * \sa getSampleNum()
   */
  bool setSampleNum(int);

protected:
  // Default constructor is  protected so that the main can't
  // create an object of the Ranger class but derived classes
  // can still use it.
  Ranger();

  // Sensor attributes
  std::string model_;           //!<Model name of sensor
  unsigned int field_of_view_;  //!<Field of view of the sensor
  unsigned int angular_res_;    //!<Angular resolution of the sensor
  double min_range_;            //!<Minimum range of the sensor
  double max_range_;            //!<Maximum range of the sensor
  int offset_;                  //!<Sensor offset
  int current_angle_;           //!<Current angle of the sensor reading
  int sample_num_;              //!<Current sample number
  int total_readings_;          //!<Total number of readings that the sensor can generate at once

  // Allows random numbers to be generatod
  double mean_;                 //!<Mean used to generate random data values
  double std_dev_;              //!<Standard deviation used to generate random data values

private:
  unsigned int seed_ = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator_;
  std::normal_distribution<double> value_distribution_;
  double generateRandNum();

};

#endif // RANGER_H
