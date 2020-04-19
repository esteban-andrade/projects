#ifndef RANGERFUSIONINTERFACE_H
#define RANGERFUSIONINTERFACE_H

#include <vector>
#include "rangerinterface.h"

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

    //! Accepts container of rangers
    virtual void setRangers(std::vector<RangerInterface*> rangers) = 0;

    //! Returns a container of fused internal data readings
    virtual std::vector<double> getFusedRangeData() = 0;

    //! Returns a container of raw data range readings 
    virtual std::vector<std::vector<double>> getRawRangeData() = 0;

    //! Sets the fusion method of the RangerFusion class
    virtual void setFusionMethod(FusionMethod) = 0;

};

#endif // RANGERFUSIONINTERFACE_H