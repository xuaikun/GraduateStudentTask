//
//  solution.h
//  
//
//  Created by Mateus Coelho on 11/07/17.
//
//

#ifndef _solution_h
#define _solution_h

#include <iostream>
#include <algorithm>    // std::random_shuffle
#include <vector>       // std::vector
#include <ctime>        // std::time
#include <cstdlib>
#include <chrono>

using namespace std;

class solution
{
public:
    //constructor
    solution(int size);
    
    //display solution
    void disp();
    
    //GA genotype
    vector<int> genotype;
    //
    vector<vector<int>> partition;
    //Fitness
    int Fitness;
};


#endif
