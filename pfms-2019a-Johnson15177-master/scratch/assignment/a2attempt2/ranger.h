#ifndef RANGER_H
#define RANGER_H

#include <string>
#include "rangerinterface.h"

using namespace std;
/*!
 *  \ingroup    Ranger
 *  \brief      Ranger derived class of Ranger Interface
 *  \details
 *  This is the derived class of Ranger Interface.\n
 *  \author     Johnsonn Nguyen
 *  \version    1
 *  \date       2019
 *  \pre        none
 *  \bug        none reported as of 2019-04-23
 *  \warning
 */
class Ranger: public RangerInterface
{
public:
  //Default constructor should set all sensor attributes to a default value
  Ranger();
  //! gets the model for laser and radar
  virtual string getModel(void) = 0;
 
  //See rangerinterface.h for more information
};

#endif // RANGER_H
