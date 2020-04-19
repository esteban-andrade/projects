#ifndef RANGERFUSION_H
#define RANGERFUSION_H

#include <vector>
#include "rangerfusioninterface.h"

class RangerFusion: public RangerFusionInterface
{
public:
  //Default constructor should set all RangerFusion attributes to a default value
  RangerFusion();

  //! Pushes the sensors into the rangers vector container
  /*!
    @param[in] rangers_
  */
  void setRangers(std::vector<RangerInterface*> rangers);

  //! Fuses the Data obtained by the generated data and fuses depending on user input 
  /*!
    @param[in] laserAngle_
    @param[in] radarData_
    @param[in] radarNumber_

    \return fusedData_
    \sa getFusedRangeData() & generateData() in ranger.h
  */
  std::vector<double> getFusedRangeData();

  //! Obtains random data for the sensors within its parameters and corresponding to an angle
  /*! 
      \sa generateData() in ranger.h
      \return data_
  */
  std::vector<std::vector<double>> getRawRangeData();

  //! Sets the Fusion Method decided by the user
  /*!
    @param[out] input_
  */
  void setFusionMethod(FusionMethod);

  //! Sends the angles of the objects 
  /*!
    \return angles_
  */
  std::vector<std::vector<std::vector<double>>> getAngles();
private:
  std::vector<std::vector<double>> data_;
  std::vector<std::vector<std::vector<double>>> angles_;
  std::vector<RangerInterface*> rangers_;
  std::vector<double> laserData_; //accesses laser data
  std::vector<double> laserAngle_;
  std::vector<double> radar0_;
  std::vector<std::vector<double>> radar1_;
  std::vector<double> radarData_;
  std::vector<std::vector<double>> radarNumber_ ;
  int radarOffset;
  int radarAngRes;
  int laserRes;
  FusionMethod input_;
};

#endif // RANGERFUSION_H
