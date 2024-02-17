#include <iostream>
#include <cmath>
#include <numeric>
#include <omp.h>

constexpr int max_tasks = 30;
constexpr int k = 100;

long long counts[max_tasks][k];

void recurse_ind(const int prod, const int task, const int n_tasks) {
    if(prod > k) return;
    for (int i = 1; i <= k; ++i) {
        const int x = i * prod;
        if (task == n_tasks) {
            if (x <= k) counts[task-1][x-1]++;
            else return;
        }
        else {
            recurse_ind(x, task+1, n_tasks);
        }
    }
}

void get_harmonic_counts() {
    #pragma omp parallel
    {
        int begin = omp_get_thread_num();
        int step = omp_get_num_threads();
        for (int n_tasks = begin; n_tasks < max_tasks; n_tasks += step) {
            recurse_ind(1, 1, n_tasks+1);
            for (int i = 1; i < k; ++i) {
                counts[n_tasks][i] += counts[n_tasks][i-1];
            }
        }
    }
    for (int n_tasks = 0; n_tasks < max_tasks; ++n_tasks) {
        for (int i = 0; i<k; ++i) {
            std::cout << n_tasks+1 << ' ' << i+1 << ' ' << counts[n_tasks][i] << std::endl;
        }
    }
}

int recurse(const int a, const int prod, const int level, const int max_level) {
    int count = 0;
    if(prod > a) return 0;
    for (int i = 1; i <= a; ++i) {
        if (level == max_level) {
            if (i * prod <= a) ++count;
            else break;
        }
        else {
            count += recurse(a, i * prod, level+1, max_level);
        }
    }
    return count;
}

int main(int argc, char * argv[]) {
    // int a = atoi(argv[1]);
    // int b = atoi(argv[2]);
    // long long count = 0;
    // for (int i = 1; i <= a; ++i) {
    //     for (int j = 1; j <= a; ++j) {
    //         if(i*j > a) break;
    //         for (int k = 1; k <=a; ++k) {
    //             if(i*j*k > a) break;
    //             for (int l = 1; l <= a; ++l) {
    //                 if(i*j*k*l > a) break;
    //                 for (int m = 1; m <= a; ++m) {
    //                     if(i*j*k*l*m > a) break;
    //                     for (int n = 1; n <= a; ++n) {
    //                         if(i*j*k*l*m*n > a) break;
    //                         ++count;

    //                     }
    //                 }

    //             }
    //         }
    //     }
    // }

    // std::cout << count << std::endl;

    get_harmonic_counts();
    //std::cout << recurse(k, 1, 1, max_tasks) << std::endl;

    // for (int n_tasks = 5; n_tasks <= 6; ++n_tasks) {
    //     for (int k = 48; k <= 50; ++k) {
    //         std::cout << recurse(k, 1, 1, n_tasks) << ' ';
    //         std::cout << counts[n_tasks-1][k-1] << std::endl;
    //     }
    // }
    

    // int lb = 0;
    // for (int i = 1; i <= a; ++i) {
    //     lb += (int)pow((int)n,(int)floor(log2(i)));
    // }
    // std::cout << lb << std::endl;


}