#include "task.h"

#include <iostream>

//Print task system information
void print_info(const Tasks & tasks) {
    

    for(Task task : tasks) {
        std::cout << task.i.t_min << ' ' << task.i.t_max << ' '
                  << task.t << ' ' << task.c << ' ' << task.e << ' '
                  << task.u_min() << ' ' << task.u_max() << ' ' << task.u()
                  << std::endl;
    }
}