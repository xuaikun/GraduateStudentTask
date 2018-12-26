#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <ctime>
#include "Graph.h"

using namespace std;

int main(int argc, char * argv[])
{
    Graph * G = NULL;
    char c;

    ifstream infile(argv[1]);
    if (!infile)
    {
        cout << "ARQUIVO INDISPONIVEL" << endl;
        return 3;
    };

    string line;
    while (getline(infile, line))
    {
        istringstream i(line);

        char control;
        i >> control;
        if (control == 'p')
        {
            string s;
            int verts, a;
            i >> s >> verts >> a;
            G = new Graph(verts);

        }
        else if (control == 'e' && G != NULL)
        {
            vector<int> ins;
            int v1, v2;
            i >> v1 >> v2;
            ins.push_back(v1-1);
            ins.push_back(v2-1);
            G->insertEdge(ins);
        }
    }
    infile.close();

    if (G == NULL)
    {
        cout << "ALERTA! GRAFO NAO INICIALIZADO" << endl;
        return 4;
    }

    cout << "Verts: " << G->V << endl;
    cout << "Edges: " << G->E << endl << endl;

    vector<vector<int> > part;
    part = G->cliquePartBTGA(stoi(argv[2]), stoi(argv[3]));
    
    ofstream outfile;
    outfile.open("partition.txt");

    outfile << part.size() << endl;
    for (int i = 0; i < part.size(); i++)
    {
        outfile << part[i].size() << endl;
        for (int j = 0; j < part[i].size(); j++)
        {
            outfile << part[i][j] << " ";
        }
    }
    outfile.close();

    delete G;

    return 0;
}
