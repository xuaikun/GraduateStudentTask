//
//  solution.cpp
//  
//
//  Created by Mateus Coelho on 11/07/17.
//
//

#include "solution.h"

using namespace std;

int seed (int i) {return (chrono::system_clock::now().time_since_epoch().count())%i;}


solution::solution(int size)
{
    srand ( unsigned ( std::time(0) ) );
    vector<int> myvector;
    
    for (int i=0; i<size; ++i) genotype.push_back(i);
    
    // using built-in random generator:
    random_shuffle ( genotype.begin(), genotype.end(), seed);
}

void solution::disp()
{
    for (int i = 0; i < genotype.size(); i++)
    {
        cout << genotype[i] << " ";
    }
    cout << endl;
}