#ifndef RANGERFUSION_H
#define RANGERFUSION_H

#include <vector>

#include "rangerfusioninterface.h"
#include "rangerinterface.h"

/*!
 *  \ingroup    RangerFusion
 *  \brief      RangerFusion derived class of RangerFusionInterface
 *  \details
 *  This is the derived class of RangerFusionInterface.\n
 *  \author     Johnsonn Nguyen
 *  \version    1
 *  \date       2019
 *  \pre        none
 *  \bug        none reported as of 2019-04-23
 *  \warning
 */

class RangerFusion: public RangerFusionInterface
{
public:
  //Default constructor should set all RangerFusion attributes to a default value
  RangerFusion();
  //! stores the sensor objects inside the vector
  void setRangers(std::vector<RangerInterface*> rangers);
  //! generates the data for each sensor and the fused vector as well
  std::vector<std::vector<double>> getRawRangeData();
  //! returns the fused vector
  std::vector<double> getFusedRangeData();
  //! provides calculations of each method of min, max and average, replaces fused vector with values depending on the method chosen
  void setFusionMethod(FusionMethod);

  //See rangerfusioninterface.h for more information
  //! sets the method to be used
  bool takeInput(unsigned int input);

private:
  //This is to cater for getRawRangeData (which generates the raw data))
  std::vector<std::vector<double>> data_; //!< stores the random vales, the raw values of each sensor
  std::vector<RangerInterface*> rangers_;   //!< stores the sensor objects inside the vector
  std::vector<double> fused;    //!< stores the fused values
  int laser_index_; //!< stores the laser index number
  int radar_index_; //!< stores the radar index number
  int fusion_method_;   //!< stores the fusion method
};

#endif // RANGERFUSION_H
