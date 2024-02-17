#pragma once

#include "task.h"

//Denotes a projection to the interval i with multiplier a from the previous interval
struct Projected_Harmonic_Zone {
    Interval i;
    int a;
};

//Denotes an interval i on the last task,
//the result of projection from the first according to the list of multipliers
struct Projected_Harmonic_Interval {
    Interval i;
    std::vector<int> multipliers;
};

//The complete sequence of projected harmonic intervals
struct Chain {
    std::vector<Projected_Harmonic_Zone> harmonics;

    //The maximum and minimum utilizations and objective values 
    float u_min, u_max, O_min, O_max;

    //Constant values to compute objective values
    float A, B, Y;

};

//A region in the lookup table
struct Region {

    //Bounds on the region
    float lb, ub;

    //Pointer to the corresponding chain
    struct Chain * chain;

    //Sort
    friend bool operator < (const Region & a, const float u);
};

//Encapsulates the functionality for harmonic elastic adjustment
class Harmonic_Elastic {

    Tasks tasks;
    std::vector<Chain> chains;
    std::vector<Region> regions;
    float u_min;



    void generate_intersections();

    void assign_periods(const Chain & chain, float u_max);


public:
    const inline float get_u_min() { return u_min; }
    const inline Tasks & get_tasks() { return tasks; }
    Chain * assign_periods_slow(float u_max);
    Chain * assign_periods(float u_max);
    bool generate();
    void add_task(Task t);
    Harmonic_Elastic(int n_tasks);
};

bool find_harmonic(Tasks & taskset);
bool verify_harmonic(const Tasks & taskset);