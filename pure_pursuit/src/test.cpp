#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>

std::string csv_path = "../racelines/smooth_trajectory.csv";

int n_pathpoints = 100;

struct PathPoint
{
    double x, y, l;

    PathPoint(double x, double y) : x(x), y(y), l(std::sqrt(std::pow(x, 2) + std::pow(y, 2))){}

    // Copy Constructor to debug inefficiencies of std::vector
    PathPoint(const PathPoint& other) : x(other.x), y(other.y)
    {
        std::cout << "Copied!" << std::endl;
    }
};


int main()
{
    // Open the csv
    std::ifstream csv(csv_path);

    if(!csv.is_open())
    {
        std::cerr << "Error: Could Not Open the File" << std::endl;
        return -1;
    }
 
    // Create a vector to hold PathPoints
    std::vector<PathPoint> pathpoints;
    pathpoints.reserve(n_pathpoints);

    std::string row, x_str, y_str;

    for(int i = 0; i < n_pathpoints; i++)
    {
        // Read one line (x, y)
        std::getline(csv, row, '\n');
        std::stringstream ss(row);

        for(int j = 0; j < 2; j++)
        {
            // Extract x and y in two iterations
            if(j == 0)
            {
                std::getline(ss, x_str, ',');
            }
            else if (j == 1)
            {
                std::getline(ss, y_str);
            }
        }

        // Push the new element into the vector
        pathpoints.emplace_back(std::stod(x_str), std::stod(y_str));
    }

    std::cout << "Elements: " << pathpoints[0].x << ", " << pathpoints[0].y << ", " << pathpoints[0].l << std::endl;

    // std::cout << "Size: " << pathpoints.size() << std::endl;

    return 0;
}