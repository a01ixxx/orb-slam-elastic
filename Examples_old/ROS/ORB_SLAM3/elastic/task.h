#pragma once
#include <vector>

//Denotes a continuous interval of periods
struct Interval {
    float t_min, t_max;
};

//Parameters of a task
struct Task {

    //The acceptable range of assignable periods
    Interval i;

    //Assigned period, execution time, elasticity
    float t, c, e;

    //Compute utilization values
    inline float u_min() const {
        return c/(float)i.t_max;
    }

    inline float u_max() const {
        return c/(float)i.t_min;
    }

    inline float u() const {
        return c/(float)t;
    }

    //Construct task according to period range, execution time, and elasticity
    Task(float t_min, float t_max, float _c, float _e) :
        i {t_min, t_max},
        c {_c},
        e {_e} {}

    bool operator < (const Task & t) {
        return i.t_min < t.i.t_min;
    }
};

// bool operator < (const Task & a, const Task & b) {
//     return a.i.t_min < b.i.t_min;
// }

//A system of tasks
using Tasks = std::vector<Task>;

//Print information about a system of tasks
void print_info(const Tasks & tasks);


