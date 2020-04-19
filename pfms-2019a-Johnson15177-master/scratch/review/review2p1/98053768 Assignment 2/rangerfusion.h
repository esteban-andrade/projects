#ifndef RANGERFUSION_H
#define RANGERFUSION_H

#include <vector>
#include "rangerfusioninterface.h"
#include "rangerinterface.h"

/*!
 * \ingroup rfi
 *  \author    Samuel Funk
 *  \version   1.0
 *  \date      2019-04-23
 *  \bug       none reported as of 2019-04-23
 *  \details
 *  Creates an object which enables fusion of readings between one laser and several radars.\n\n
 * USAGE:
 * 1. Instantiate an object of the RangerFusion class.
 * 2. Optionally call setFusionMethod() to select a fusion
 * method. The methods are: minimum value (FUSION_MIN),
 * maximum value (FUSION_MAX) and average value (FUSION_AVG).
 * 3. Use setRangers() to set the rangers which will be used to gather data.
 * 4. Use getRawRangeData() to create data to be fused.
 * 5. Use getFusedRangeData() to fused the created data.\n
 */

class RangerFusion: public RangerFusionInterface
{
public:
  //Default constructor should set all RangerFusion attributes to a default value
  RangerFusion();

  // Accepts container of rangers - as per requirement C2
  /*! Accepts a vector of rangers whose data is to be fused.\n
   * As the data is fused with respect to the laser, the vector can
   * only contain ONE LASER.\n
   * There is no limit to the number of radars that can be included.\n
   * This function must be called at least once before calling getRawRangeData().
   @param[in] vector of RangerInterface objects\n
   * \sa getFusedRangeData(), getRawRangeData()
   */
  void setRangers(std::vector<RangerInterface*> rangers);

  // Returns a container of fused internal data readings - as per requirement C5
  /*! Returns a vector of fused internal data readings.\n
   * Data is fused using the method that is specified with setFusionMethod(),
   * or by average value if none is specified.\n
   * When data is fused, the position of each laser reading is calculated, and compared
   * with the positions of each radar reading. If one or more radar readings are
   * recorded at the same position of the laser reading, an operation is performed in
   * accordance with the specified method. This is repeated for each laser reading.\n
   * The fusion methods are as follows:\n
   * - Minimum value (FUSION_MIN): The lowest data value is taken from all readings at
   * the same location.\n
   * - Maximum value (FUSION_MAX): The largest data value is taken from all readings at
   * the same location.\n
   * - Minimum value (FUSION_AVG): The average of all the readings at the same location
   * is taken.\n\n
   * Radar readings that do not correspond to any laser reading location (including those
   * that are outside the range of the laser readings) will not be fused with any laser reading.\n
   * NOTE: Rangers must be supplied and data must be generated before calling this function
   * i.e. must be called AFTER calling setRangers() and getRawRangeData() at least once.\n\n
   * \return Container of fused data.
   * \sa setRangers(), setFusionMethod, getRawRangeData()
   */
  std::vector<double> getFusedRangeData();

  // Returns a container of raw data range readings - as per requirement C4
  /*! Returns a vector of raw data readings that have been obtained from the rangers
   * that have been provided in the setRangers() function.\n
   * Rangers must be supplied before calling this function i.e. setRangers() must
   * be called at least once before calling this function.\n
   * This data can then be fused using getFusedRangeData()
   * \return Container of raw data.
   * \sa setRangers(), getRawRangeData()
   */
  std::vector<std::vector<double>> getRawRangeData();

  // Sets the fusion method of the RangerFusion class - as per requirement C6
  /*! Sets the method to fuse data in getFusedData().\n
   * Argument must be of FusionMethod type i.e. either FUSION_MIN, FUSION_MAX OR FUSION_AVG\n
   * \sa getFusedData()
   */
  void setFusionMethod(FusionMethod);

  /*! Returns the method to fuse data that is currently set.
   * \return Value of FusionMethod type i.e. either FUSION_MIN, FUSION_MAX OR FUSION_AVG\n
   * \sa setFusedData()
   */
  FusionMethod getFusionMethod(void);

  /*!
   * \return Current sample number.
   * \sa setSampleNum()
   */
  int getSampleNum(void);

  /*! Sets the current sample number.\n
   @param[in] sample_num Desired sample number value.\n
   * \return True if the set number is valid (i.e greater than zero); false otherwise.
   * \sa getSampleNum()
   */
  bool setSampleNum(int);


private:
  FusionMethod fusion_method_;
  std::vector<RangerInterface*> rangers_;

  // Stores the index of the laser in the rangers_ vector
  int laser_index_;

  //This is to cater for getRawRangeData (which generates the raw data))
  std::vector<std::vector<double>> data_;

  // Stores the i
  int sample_num_;

};

#endif // RANGERFUSION_H
