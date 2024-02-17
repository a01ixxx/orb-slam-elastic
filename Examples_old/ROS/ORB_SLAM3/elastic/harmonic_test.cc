#include "task.h"

#include <iostream>
#include <cmath>


std::vector<Chain> enumerate_harmonics(const Tasks & taskset, const Harmonic harmonic, const int i, Chain chain) {

    chain.harmonics.push_back(harmonic);

    //No more intervals to forward project
    if (i == (int)taskset.size()) return {chain};

    //Vector of chains
    std::vector<Chain> chains;

    //Get multipliers from current interval to next task's period interval
    Interval l = harmonic.i;
    Interval r = taskset[i].i;
    int lb = (int) std::ceil(r.t_min/l.t_max);
    int ub = (int) std::floor(r.t_max/l.t_min);

    //Iterate over multipliers
    for (int a = lb; a <= ub; ++a) {

        //Get overlapping projection region
        Interval overlap;
        overlap.t_min = std::max(l.t_min * a, r.t_min);
        overlap.t_max = std::min(l.t_max * a, r.t_max);
        Harmonic h {overlap, a};

        //Project region to end
        auto new_chains = enumerate_harmonics(taskset, h, i+1, chain);
        for (auto new_chain: new_chains) {
            chains.push_back(new_chain);
        }
    }

    return chains;

}

void backpropagate(std::vector<Chain> & chains) {
    for (auto & full_chain : chains) {
        auto & chain = full_chain.harmonics;
        for(int i = chain.size() - 1; i > 0; --i) {
            Interval & l = chain[i-1].i;
            Interval & r = chain[i].i;
            // std::cout << l.t_min << ' ' << l.t_max << " | " << r.t_min << ' ' << r.t_max << ' ' << chain[i].a << " | ";
            l.t_min = r.t_min / chain[i].a;
            l.t_max = r.t_max / chain[i].a;
            // std::cout << l.t_min << ' ' << l.t_max << std::endl;
        }
        for(int i = 1; i < chain.size(); ++i) {
            chain[i].a *= chain[i-1].a;
        }
    }
}

// Harmonics harmonic_range(Interval a, Interval b) {
//     Harmonics harmonics;
//     int lb = b.t_min/a.t_max + (b.t_min % a.t_max ? 1 : 0);
//     int ub = b.t_max/a.t_min;

//     for (int i = lb; i <= ub; ++i) {
//         harmonics.push_back(i);
//     }
//     return harmonics;
// }

// void get_objective(Tasks & taskset, Harmonics & harmonics, const float u, float & t) {
//     int t_min = taskset[0].t_min;
//     int t_max = taskset[0].t_max;
//     for (int i = 1; i < taskset.size(); ++i) {
//         t_min = std::max(t_min, taskset[i].t_min * harmonics[i-1]);
//     }

// }

// void print_harmonics(Task & t1, Task & t2) {

//     for(int h : harmonic_range(t1.i, t2.i)) {
//         std::cout << h << ' ';
//     }
//     std::cout << std::endl;
// }

// int main() {
//     Task t1 {2, 4, 1, 1, 1};
//     Task t2 {1, 5, 1, 1, 1};
//     Task t3 {2, 8, 1, 1, 1};
//     Task t4 {3, 9, 1, 1, 1};
//     Task t5 {4, 7, 1, 1, 1};
//     Task t6 {6, 11, 1, 1, 1};
//     Task t7 {15, 27, 1, 1, 1};

//     print_harmonics(t1,t2);
//     print_harmonics(t1,t3);
//     print_harmonics(t1,t4);
//     print_harmonics(t1,t5);
//     print_harmonics(t1,t6);
//     print_harmonics(t1,t7);

// }

void print_harmonics(const std::vector<Chain> & harmonics) {
    

    for(auto chain : harmonics) {
        for (Harmonic h : chain.harmonics) {
            std::cout << h.a << ' ' << h.i.t_min << ' ' << h.i.t_max << " | ";
        }
        std::cout << std::endl;
    }
}

void print_info(const std::vector<Chain> & harmonics) {
    

    for(auto chain : harmonics) {
        std::cout << chain.u_min << ' ' << chain.u_max << ' '
                  << chain.O_min << ' ' << chain.O_max << ' '
                  << chain.A << ' ' << chain.B << std::endl;
    }
}

void print_info(const Tasks & tasks) {
    

    for(Task task : tasks) {
        std::cout << task.i.t_min << ' ' << task.i.t_max << ' '
                  << task.t << ' ' << task.c << ' ' << task.e << ' '
                  << task.u_min() << ' ' << task.u_max() << ' ' << task.u()
                  << std::endl;
    }
}

float compute_loss(Chain & chain, const float U) {

    return chain.A * U * U - chain.B * U;
}

float compute_loss(const Tasks & taskset, const Chain & chain, const float U) {
    float T = 0;
    for (int i = 0; i < taskset.size(); ++i) {
        T += taskset[i].c/(float)chain.harmonics[i].a;
    }
    T /= U;

    float O = 0;
    for (int i = 0; i < taskset.size(); ++i) {
        float o = taskset[i].u_max() - taskset[i].c / (chain.harmonics[i].a * T);
        O += o * o / taskset[i].e;
    }

    return O;
}

void compute_chain_properties(const Tasks & taskset, std::vector<Chain> & chains) {

    for (Chain & chain : chains) {

        float u_min = 0;
        float u_max = 0;

        float Y = 0;
        float A = 0;
        float B = 0;

        for (int i = 0; i < (int)chain.harmonics.size(); ++i) {
            u_min += taskset[i].c/chain.harmonics[i].i.t_max;
            u_max += taskset[i].c/chain.harmonics[i].i.t_min;
            
            float y = taskset[i].c/(float)chain.harmonics[i].a;
            Y += y;
            float u_i_max = taskset[i].u_max();
            A += y * y / taskset[i].e;
            B += 2 * y * u_i_max / taskset[i].e;
        }

        float X = 1/Y;
        A = A * X * X;
        B = B * X;

        chain.A = A;
        chain.B = B;
        chain.u_min = u_min;
        chain.u_max = u_max;


        //Assumes the minimum objective corresponds with maximum utilization
        //TODO: May not be the case if c depends on harmonic range
        chain.O_max = compute_loss(chain, u_min);
        chain.O_min = compute_loss(chain, u_max);

    }
}

void generate_intersections(float u_max, const std::vector<Chain> & chains) {
    std::vector<Region> regions;
    const Chain & chain = chains[0];
    regions.push_back({0, chain.u_min, true, chain.O_max, 0});
    regions.push_back({chain.u_min, chain.u_max, false, chain.A, chain.B});
    regions.push_back({chain.u_max, u_max, true, chain.O_min, 0});

    for(int i = 1; i < chains.size(); ++i) {
        const Chain & chain = chains[i];
        
    }
}

void intersections(struct Region, Chain & chain) {

}


int main() {
    Task t1 {5,6,1,0.3,3};
    Task t2 {12,17,1,0.7,4};
    Task t3 {23,32,1,0.1,1};
    Tasks tasks = {t1, t2, t3};
    auto chains = enumerate_harmonics(tasks, {tasks[0].i, 1}, 1, {});
    print_harmonics(chains);
    backpropagate(chains);
    std::cout << '\n';
    print_harmonics(chains);
    std::cout << '\n';
    compute_chain_properties(tasks, chains);
    print_info(chains);
    print_info(tasks);
}