// Fuzzy.h



//
// Created by victor on 9/30/19.
//

#ifndef ROBOT_CONTROL_FUZZY_H
#define ROBOT_CONTROL_FUZZY_H

#include <iostream>
#include <vector>

class Fuzzy {
    
  public:
    Fuzzy();
    ~Fuzzy();
    std::pair<float, float>     rc_fuzzy(std::pair<float,float>);
    std::pair<float, float>     fuzzy(std::pair<float, float>);
    std::pair<float, float>     find_shortest_laser(std::vector<std::pair<float, float>> laserLines);
    bool                        wallFound(std::pair<float, float> scan_of_interest);

    
};



#endif //ROBOT_CONTROL_FUZZY_H

