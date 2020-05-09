#include <math.h>
#include <iostream>
#include <deque>
#include <cmath>
#include <iostream>
#include "seek.h"

Seek::Seek(){
  
}

int Seek::circle_circle_intersection(GlobalOrd friendly, GlobalOrd base, double bogieToFriendly,
                               double bogieToBase, Pose &bogie0, Pose &bogie1)
{
  double a, dx, dy, d, h, rx, ry;
  double x2, y2;
  
  dx = friendly.x - base.x;
  dy = friendly.y - base.y;

  d = hypot(dx,dy); 

  if (d > (bogieToFriendly + bogieToBase))
  {
    return 0;
  }
  if (d < fabs(bogieToBase - bogieToFriendly))
  {
    return 0;
  }

  a = ((bogieToBase*bogieToBase) - (bogieToFriendly*bogieToFriendly) + (d*d)) / (2.0 * d) ;

  x2 = base.x + (dx * a/d);
  y2 = base.y + (dy * a/d);

  h = sqrt((bogieToBase*bogieToBase) - (a*a));

  rx = -dy * (h/d);
  ry = dx * (h/d);

  bogie0.position.x = x2 + rx;
  bogie0.position.y = y2 + ry;
  bogie1.position.x = x2 - rx;
  bogie1.position.y = y2 - ry;

  return 1;
}

void Seek::findPosition(Pose& bogie0, Pose& bogie1){
        xPosition0.push_front(bogie0.position.x);
        yPosition0.push_front(bogie0.position.y);
        xPosition1.push_front(bogie1.position.x);
        yPosition1.push_front(bogie1.position.y);
        while(xPosition0.size() == 2 && yPosition0.size() == 2){
          distance0 = sqrt(pow(xPosition0.front()-xPosition0.back(), 2)+ pow(yPosition0.front()-yPosition0.back(), 2));
          distance1 = sqrt(pow(xPosition1.front()-xPosition1.back(), 2)+ pow(yPosition1.front()-yPosition1.back(), 2));
          bogie0.orientation = atan2(yPosition0.front() - yPosition0.back(), xPosition0.front() - xPosition0.back());
          bogie1.orientation = atan2(yPosition1.front() - yPosition1.back(), xPosition1.front() - xPosition1.back());
          if(bogie1.orientation <= 0){
              bogie1.orientation= 2*M_PI - fabs(bogie1.orientation);
          }
          if(bogie0.orientation <= 0){
            bogie0.orientation = 2*M_PI - fabs(bogie0.orientation);
          }
          xPosition0.clear();
          yPosition0.clear();
          xPosition1.clear();
          yPosition1.clear();
        }
}

void Seek::pushBogies(bool start,Pose& bogie0, Pose& bogie1){
      if(start){
          start = false;
          correctBogie.push_front(bogie0);
      } else {
          bogies.push_front(bogie0);
          bogies.push_front(bogie1);
      }      
}

void Seek::getPosition(Pose& bogie0, Pose& bogie1){
  if(bogies.size() == 2){
    double startVal = sqrt(pow(correctBogie.front().position.x, 2) + (pow(correctBogie.front().position.x, 2)));
    double startOrientation = correctBogie.front().orientation;

    double offset0 = fabs(startOrientation - bogie0.orientation);//bogie0.orientation);
    double offset1 = fabs(startOrientation - bogie1.orientation);//bogie1.orientation);
    if(distance0 < distance1 && offset0 < offset1){
      correctBogie.push_front(bogie0); //0
    }else if(distance0 > distance1 && offset0 < offset1){ 
      correctBogie.push_front(bogie1); //1
    }else if(distance0 > distance1 && offset0 > offset1){               
      correctBogie.push_front(bogie1); //1
    }else if(distance0 < distance1 && offset0 > offset1){
      correctBogie.push_front(bogie0); //0
    }

    bogies.clear();
    correctBogie.pop_back();
    }    
}

std::deque<Pose> Seek::getBogies(){
  return bogies;
}

std::deque<Pose> Seek::getCorrectBogie(){
  return correctBogie;
};
      