

/**
* Innovative Global Solutions - IGS
* Infant Incubator Project
* 
* Header file defining class for bounds 
*/


#ifndef BOUND_HPP
#define BOUND_HPP

class Bound{
  
  public:
    //upper bound
    double upperBound;

    //lower bound
    double lowerBound;

    //soft offset triggers as indicated by proportion. Allows for asymmetric
    double offsetProportion[2] = {0,1};

    /**
    * Initializes a bound with no soft limits
    *
    * @param lowerBound the lower bound hard limit
    * @param upperBound the upper bound hard limit
    *
    * @return A bound object
    */
    Bound(double lowerBound, double upperBound);

    /**
    * Initializes a bound with no soft limits
    *
    * @param lowerBound the lower bound hard limit
    * @param upperBound the upper bound hard limit
    * @param offset the offset levels desired for soft limits
    *
    * @return A bound object
    */
    Bound(double lowerBound, double upperBound, double offset[2]);

    /**
    * Setter function for changing the lower bound
    *
    * @param bound new lower bound
    */
    void setLowerBound(double bound);
    void setUpperBound(double bound);
    void setOffset(double offset[2]);

    /**
    * Function to find the status based on a provided 
    *
    */
    int getStatus(double currentVal);

};

#endif