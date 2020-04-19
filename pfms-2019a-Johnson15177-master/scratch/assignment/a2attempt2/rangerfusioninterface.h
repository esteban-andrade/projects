#ifndef RANGERFUSIONINTERFACE_H
#define RANGERFUSIONINTERFACE_H

#include <vector>
#include "rangerinterface.h"

typedef enum {
  FUSION_MIN,
  FUSION_MAX,
  FUSION_AVG
} FusionMethod; /*!< Available data fusion methods - as per requirement C1*/

/*!
 *  \ingroup    RangerFusion
 *  \brief      RangerFusionInterface base class
 *  \details
 *  This is the base class.\n
 *  \author     Johnsonn Nguyen
 *  \version    1
 *  \date       2019
 *  \pre        none
 *  \bug        none reported as of 2019-04-23
 *  \warning
 */

// The RangerFusionInterface is a class which specifies the minimum
// required interface for your RangerFusion class your ranger fusion
// class must inherit from it
class RangerFusionInterface
{
public:
    RangerFusionInterface();

    // Accepts container of rangers - as per requirement C2 
    //! stores the sensor objects inside the vector
    virtual void setRangers(std::vector<RangerInterface*> rangers) = 0;

    // Returns a container of fused internal data readings - as per requirement C4
    //! returns the fused vector
    virtual std::vector<double> getFusedRangeData() = 0;

    // Returns a container of raw data range readings - as per requirement C5 
    //! generates the data for each sensor and the fused vectir as well
    virtual std::vector<std::vector<double>> getRawRangeData() = 0;

    // Sets the fusion method of the RangerFusion class - as per requirement C6
    //! provides the calculations of each method of min, max and average, replaces fused vector with values depending on the method chosen
    virtual void setFusionMethod(FusionMethod) = 0;

};

#endif // RANGERFUSIONINTERFACE_H
