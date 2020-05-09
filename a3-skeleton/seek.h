#ifndef SEEK_H
#define SEEK_H

#include <iostream>
#include <cmath>
#include "lib/types.h"

class Seek{
  public:
    /*!
    Creates the seeker object to find the points
    */
    Seek();
    /*!
    Grabs the two coordinates from the radius of the Base & Friendly
    @param[in] GlobalOrd friendly, GlobalOrd base, double bogieToFriendly, double bogieToBase, Pose &bogie0, Pose &bogie1
    */
    int circle_circle_intersection(GlobalOrd friendly, GlobalOrd base, double bogieToFriendly,
                               double bogieToBase, Pose &bogie0, Pose &bogie1);
    /*!
    Obtains the pose of the bogies
    @param[in] Pose& bogie0, Pose& bogie1
    */
    void findPosition(Pose&, Pose& );
    /*!
    Pushes the bogie data to find the correct bogie
    @param[in] bool start,Pose& bogie0, Pose& bogie1
    */
    void pushBogies(bool ,Pose& , Pose& );
    /*!
    Obtains the correct position
    */
    void getPosition(Pose&, Pose& );
    /*!
    \return bogies
    */
    std::deque<Pose> getBogies();
    /*!
    \return correctBogie
    */
    std::deque<Pose> getCorrectBogie();
      
  private:
    std::deque<Pose> bogies;
    std::deque<Pose> correctBogie;
    double distance0;
    double distance1;
    std::deque<double> xPosition0;
    std::deque<double> yPosition0;
    std::deque<double> xPosition1;
    std::deque<double> yPosition1;
};

#endif