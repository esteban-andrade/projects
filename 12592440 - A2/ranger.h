#ifndef RANGER_H
#define RANGER_H
#include <vector>
#include "rangerinterface.h"

class Ranger: public RangerInterface
{
  //Default constructor should set all sensor attributes to a default value
  public:
    Ranger();
    Ranger(int angleRes, int offset, double fieldOfView, double maxDist, double minDist, int numSamples);

    //! generates random data for chosen sensor
    /*!
        \return vector 
        @param[out] rangerData_
    */
    std::vector<double> generateData();

    //! gets the Angular Resolution for the sensor either 30 degrees for radar or 10 degrees or 30 degrees for laser 
    /*!
        \return angular resolution
    */
    unsigned int getAngularResolution(void);
    //! gets the Field of View for the sensor either 180 degrees for laser or 60 degrees for radar.
    /*!
        \return field of view
    */
    unsigned int getFieldOfView(void);
    //! gets the offset for the sensor, either default or decided by user
    /*!
        \return offset
    */
    int getOffset(void);
    //! gets the Maximum distance for the sensor
    /*!
        \return maximum distance
    */
    double getMaxRange(void);
    //! gets the Minimum distance for the sensor
    /*!
        \return minimum distance
    */
    double getMinRange(void);

    //! Sets the Angular Resolution for chosen sensor.
    /*!
        \return True
    */
    bool setAngularResolution(unsigned int);
    //! Sets the offset for chosen sensor.
    /*!
        \return True
    */
    bool setOffset(double);
    //! Sets the Field of View for chosen sensor but unnessary as they are fixed parameters.
    /*!
        \return True
    */
    bool setFieldOfView(unsigned int);

  protected:
    std::vector<std::vector<double>> rangerAngle_;
    int rangerOffset_; 
    unsigned int rangerFOV_; 
    unsigned int rangerAngularRes_;
    std::vector<double> rangerData_;
    double rangerMaxDist_;
    double rangerMinDist_;
    int rangerNumSamples_;

};

#endif // RANGER_H
