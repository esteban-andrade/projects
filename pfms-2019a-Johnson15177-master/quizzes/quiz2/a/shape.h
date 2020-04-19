#ifndef SHAPE_H
#define SHAPE_H

#include <chrono>
#include <random>
#include <string>
#include <chrono>
#include <random>

/*!
 *  \ingroup   ac_shape Shape
 *  \brief     Shape base class
 *  \details
 *  This class is the base class for all shapes.\n
 */
class Shape
{
public:
    Shape();

    void setCentre(double x, double y);
    std::string getDescription();
    void offset(double x, double y);
    virtual double getArea() = 0;
<<<<<<< HEAD
    //virtual destructor
    virtual ~Shape(){}
  //  RandNum(int seed);
=======
   // virtual ~Shape();
    MaxLength(int seed);
>>>>>>> c0839e28f5003efc4cf7d6a7fcec4d6577a36d5b
protected:
    std::string description_;//!< description of shape
private:
    double centreX_;//!< X coordinate of centre of shape
    double centreY_;//!< Y coordinate of centre of shape
<<<<<<< HEAD
    
    std::default_random_engine generator_;
    std::uniform_real_distribution<double> distribution_;
=======
    std::default_random_engine generator_;
    std::uniform_real_distribution<double> distribution_;
    int Max_Length_;
>>>>>>> c0839e28f5003efc4cf7d6a7fcec4d6577a36d5b
};

#endif // SHAPE_H
