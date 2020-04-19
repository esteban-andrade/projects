#ifndef RANGER_H
#define RANGER_H
#include <string>
#include <iostream>
#include <random>
#include <chrono>
#include <thread>
#include <vector>

#include "rangerinterface.h"
using namespace std;

class Ranger : public RangerInterface
{
public:

    //setter
    /**
    This function is a virtual function inherite from ranger interface and set anular resolution
    @param[in]    angular resolution
    */
    virtual bool setAngularResolution(unsigned int &angularRes) = 0;
    /**
    This function is a virtual function inherite from ranger interface and set offset angle
    @param[in]    offset angle
    */
    virtual bool setOffset(int &offSet) = 0;
    /**
    This function is a virtual function inherite from ranger interface and set field of view
    @param[in]    field of view
    */
    virtual bool setFieldOfView(unsigned int &fov) = 0;

    /**
    This function set number of samples
    */
    void setNumOfSamples() ;   // set number of samples to be used for calculation functions


    /**
    This function get number of samples
    \return The number of samples
    */
    int getNumOfSamples();
    /**
    This function get angular resolution
    \return The angular resolution
    */
    unsigned int getAngularResolution(void) ;
    /**
    This function get offset value
    \return The offset value
    */
    int getOffset(void) ;
    /**
    This function get filed of view
    \return The file of view
    */
    unsigned int getFieldOfView(void) ;
    /**
    This function get model name
    \return The model name
    */
    string getModel();

    /**
    This function sensor maximum range reading
    \return The max distance
    */
    double getMaxRange();   //get max distance value
    /**
    This function sensor minimum range reading
    \return The min distance
    */
    double getMinRange();    //get min distance value

    //! this function generate the sensor reading data.
    /*!
      \return the data reading
    */
    double getData();           //generate random number

    //! container for data
    /*!
      \return The data
    */
    vector<double> generateData();  // create vector and for sensor data

    /**
    This function put data into the vector container
    */
    virtual void putDataVec() = 0;      // put data into vector


    void clear_value();      //!< clear the generate Data vector containe

protected:
    //Default constructor should set all sensor attributes to a default value
    Ranger();
    //Default deconstructor
    ~Ranger();
    // all the protect value;
    string model_;              //!< model name
    int angularResolution_;     //!< angular resolution
    int offSet_;               //!< sensor offset angle
    double maxDistance_;      //!< maximum distance
    double minDistance_;     //!< minimum distance
    int fov_;               //!< field of view variables
    int numberOfSamples_; //!< number of samples generated
    vector<double> value;   //!< vector container for data reading

    //See rangerinterface.h for more information
};

#endif // RANGER_H
