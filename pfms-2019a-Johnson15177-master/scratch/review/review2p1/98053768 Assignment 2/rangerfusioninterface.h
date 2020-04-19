#ifndef RANGERFUSIONINTERFACE_H
#define RANGERFUSIONINTERFACE_H

#include <vector>
#include "rangerinterface.h"

/*!
 * \ingroup rfi
 *  \author    Alen Alempijevic
 *  \author    Alex Virgona
 *  \version   N/A
 *  \date      2019
 *  \bug       none reported as of 2019-04-23
 *  \brief     RangerFusionInterface base class
 *  \details
 *  This is an abstract class that stipulates the interface that the RangerFusion class uses.\n
 */

typedef enum {
  FUSION_MIN,
  FUSION_MAX,
  FUSION_AVG
} FusionMethod; /*!< Available data fusion methods - as per requirement C1*/

// The RangerFusionInterface is a class which specifies the minimum
// required interface for your RangerFusion class your ranger fusion
// class must inherit from it
class RangerFusionInterface
{
public:
    RangerFusionInterface();

    // Accepts container of rangers - as per requirement C2 
    virtual void setRangers(std::vector<RangerInterface*> rangers) = 0;

    // Returns a container of fused internal data readings - as per requirement C5
    virtual std::vector<double> getFusedRangeData() = 0;

    // Returns a container of raw data range readings - as per requirement C4
    virtual std::vector<std::vector<double>> getRawRangeData() = 0;

    // Sets the fusion method of the RangerFusion class - as per requirement C6
    virtual void setFusionMethod(FusionMethod) = 0;

};

#endif // RANGERFUSIONINTERFACE_H
