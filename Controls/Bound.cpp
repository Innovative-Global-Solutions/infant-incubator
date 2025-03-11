
/**
* Innovative Global Solutions - IGS
* Infant Incubator Project
* 
* Abstract class for bound
*/

#include "Bound.hpp"

Bound::Bound(double lowerBound, double upperBound):
  lowerBound(lowerBound), upperBound(upperBound){}

Bound::Bound(double lowerBound, double upperBound, double offset[2]):
  lowerBound(lowerBound), upperBound(upperBound){
    offsetProportion[0] = offset[0];
    offsetProportion[1] = offset[1];
  }

Bound::setLowerBound(double bound) {lowerBound = bound;}
Bound::setUpperBound(double bound) {upperBound = bound;}
Bound::setOffset(double offset[2]){
  offsetProportion[0] = offset[0];
  offsetProportion[1] = offset[1];
}

int Bound::getStatus(double currentVal) {
    if (currentVal < lowerBound)
      return -2;
    else if (currentVal < lowerBound + (upperBound - lowerBound)*offsetProportion[0])
      return -1;
    else if (currentVal > upperBound)
      return 2;
    else if (currentVal > lowerBound + (upperBound - lowerBound)*offsetProportion[0])
      return 1;
    else
      return 0;
}