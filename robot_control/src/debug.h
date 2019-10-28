//
// Created by victor on 10/7/19.
//

#ifndef ROBOT_CONTROL_DEBUG_H
#define ROBOT_CONTROL_DEBUG_H

constexpr int DEBUG = 0;

template <typename... T >
void simple_debugprint(T &&... args)
{
    if(DEBUG == 1)
    {
        std::ostringstream sstr;
        // fold expression
        ((sstr << std::dec) << ... << args);
        std::cout << sstr.str() << std::endl;
    }
}


#endif //ROBOT_CONTROL_DEBUG_H
