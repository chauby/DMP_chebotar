#include "DataRecorder.h"
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

using namespace std;
namespace dmp
{

DataRecorder::DataRecorder()
{

}

void DataRecorder::setNumDimensions(int num_dimensions_new)
{
    num_dimensions = num_dimensions_new;
    current_trajectory.resize(num_dimensions);
}

void DataRecorder::addToTrajectory(vector<DMPState> state_vec)
{
    for(int i = 0; i < state_vec.size(); i++){
        current_trajectory[i].push_back(state_vec[i]);
    }

}

vector< vector<DMPState> > DataRecorder::getCurrentTrajectory()
{
    return current_trajectory;
}

void DataRecorder::clearTrajectory()
{
    current_trajectory.clear();
    current_trajectory.resize(num_dimensions);
}

bool DataRecorder::saveWeights(const vector< vector<double> > weights, const char* file_path)
{

    FILE* f = fopen(file_path, "w");
    int i,j;
    for(i = 0; i < weights.size(); i++){
        vector<double> weights_el = weights[i];

        int size = weights_el.size();
        for(j = 0; j < size;j++){
            fprintf(f, "%.7f\n", weights_el[j]);
        }
        fprintf(f, "\n");     
    }
    fclose(f);
}


bool DataRecorder::saveTrajectories(const char* file_path)
{
    FILE* f = fopen(file_path, "w");

    int i,j;
    for(i = 0; i < current_trajectory.size(); i++){
        vector<DMPState> traj = current_trajectory[i];

        for(j = 0; j < traj.size(); j++){
            DMPState state = traj[j];
            fprintf(f, "%.7f %.7f %.7f %.7f\n", state.getTime(),
                    state.getX(), state.getXd(), state.getXdd());
        }

        fprintf(f, "\n");
    }

    fclose(f);
}


vector< vector<double> > DataRecorder::loadWeights (const char* file_path)
{
    vector< vector<double> > weights;

    FILE* f = fopen(file_path, "r");
    char line[1000];
    vector<double>* current_weights = new vector<double>;

    while(fgets(line, 1000, f) != NULL){

        if(strcmp(line, "\n") == 0){
            weights.push_back(*current_weights);
            current_weights = new vector<double>;
        }else{
            double weight;
            sscanf(line, "%lf", &weight);
            current_weights->push_back(weight);
        }
    }
    delete current_weights;
    fclose(f);

    return weights;
}

DataRecorder::~DataRecorder()
{

}

}
