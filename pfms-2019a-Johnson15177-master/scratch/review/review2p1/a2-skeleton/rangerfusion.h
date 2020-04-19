#ifndef RANGERFUSION_H
#define RANGERFUSION_H

#include <vector>
#include "rangerfusioninterface.h"
#include "rangerinterface.h"
#include "ranger.h"
#include "laser.h"
#include "radar.h"
#include "map"

class RangerFusion: public RangerFusionInterface
{
public:
  //Default constructor should set all RangerFusion attributes to a default value
  RangerFusion();

  //See rangerfusioninterface.h for more information

  /**
  This function set the ranger sensors into a vector
  @param[in] The ranger sensor objects
  */
  // Accepts container of rangers - as per requirement C2
  void setRangers(std::vector<RangerInterface *> &rangers) ;

  /**
  This function get all the ranger sensor
  \return The ranger sensor objects
  */
  // get rangers vector
  std::vector<RangerInterface*> getRangers();

  //get laser raw data
  /**
  This function get the laser object raw sensor reading
  @param[in]    laser object
  */
  void getLaserData(vector<double> lasers);

  /**
  This function get the radar raw  reading
  @param[in]    radar object
  */
  //get radar raw data
  void getRadarData(vector<double> radars);

  //get the laser object
  /**
  This function get the laser object
  @param[in]    laser object
  */
  void getLaserObject(Laser &object);

  /**
  This function get the radar object
  @param[in]    radar object
  */
  //get the radar object
  void getRadarObject(Radar &object);

  //clear object vector
  /**
  This function will clear object vector
  */
  void clear();

  /**
  This function will doing fusion process
  */
  // fusion process
  void fusion();

  /**
  This function will fuse data into container
  */
  // set fuse data into container
  void setFuseRangeData();

  /**
  This function Returns a container of fused internal data readings
  \return  a container of fused internal data readings
  */
  // Returns a container of fused internal data readings - as per requirement C4
  std::vector<double> getFusedRangeData();


  /**
  This function Returns a container of raw data range readings
  \return   a container of raw data range readings
  */
  // Returns a container of raw data range readings - as per requirement C5
  std::vector<std::vector<double>> getRawRangeData();

  /**
  This function Sets the fusion method of the RangerFusion class
   @param[in]  method of min/max/average
  */
  // Sets the fusion method of the RangerFusion class - as per requirement C6
  void setFusionMethod(int &method) ;

protected:
  //This is to cater for getRawRangeData (which generates the raw data))
  std::vector<std::vector<double>> data_; //!< This is to cater for getRawRangeData (which generates the raw data))
  //fuse data container
  vector<double> fuseData; //!< fuse data container
  // fusion method
  FusionMethod method_ = FUSION_MAX ; //!<  fusion method
  // vector of ranger object
  std::vector<RangerInterface*> rangers_; //!< vector of ranger object
  //laser data vetor
  std::vector<vector<double>> laser_data_;//!<  laser data vetor
    //radar data vector
  std::vector<vector<double>> radar_data_;//!<    radar data vector
  // laser object
  std::vector<Laser> laser_object_;//!< laser object
  // radar object
  std::vector<Radar> radar_object_;//!< radar object

  //map map
  std::map<double,double> maps;//!< map container maps
};

#endif // RANGERFUSION_H
