// wind_data_processor.cpp
#include "wind_data_processor.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <limits>
#include <future>
#include <chrono>
#include <filesystem> // remove after testing
#include <nanoflann.hpp> // Include nanoflann for kd-tree implementation

namespace WindDataProcessor {

// Constructor for Array3D that reads CSV and builds the kd-tree
    Array3D::Array3D(const std::string& csvFile) {
        this->csvFileName = csvFile;
        // csv will have x, y, z, u, v, w: representing the position and wind values
        // Determine the dimensions of the 3D array
        determineDimensions();
        // Read the CSV file and construct a kd-tree from the 3D array
        loadData();
        constructKDTree();
    }

// Function to determine the dimensions of the 3D array
    void Array3D::determineDimensions() {
        // Example: Reading the CSV file to determine dimensions.
        std::ifstream file(csvFileName);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open CSV file");
        }

        double x, y, z, u, v, w;
        double minX = std::numeric_limits<double>::max();
        double minY = std::numeric_limits<double>::max();
        double minZ = std::numeric_limits<double>::max();
        double maxX = std::numeric_limits<double>::lowest();
        double maxY = std::numeric_limits<double>::lowest();
        double maxZ = std::numeric_limits<double>::lowest();

        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            if (ss >> x >> y >> z >> u >> v >> w) {
                minX = std::min(minX, x);
                minY = std::min(minY, y);
                minZ = std::min(minZ, z);
                maxX = std::max(maxX, x);
                maxY = std::max(maxY, y);
                maxZ = std::max(maxZ, z);
            }
        }

        file.close();

        this->minX = minX;
        this->minY = minY;
        this->minZ = minZ;
        this->maxX = maxX;
        this->maxY = maxY;
        this->maxZ = maxZ;
    }

// Function to load data from the CSV file into the 3D array and windValues
    void Array3D::loadData() {
        std::ifstream file(csvFileName);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open CSV file");
        }

        double x, y, z, u, v, w;
        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            if (ss >> x >> y >> z >> u >> v >> w) {
                // Store the 3D point and corresponding wind vector
                positions.push_back({x, y, z});
                windValues.push_back({u, v, w});
            }
        }

        file.close();
    }

// Function to construct the kd-tree from the loaded 3D points
    void Array3D::constructKDTree() {
        // Build the kd-tree using nanoflann
        kdTree = std::make_unique<KDTree>(3, *this, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
        kdTree->buildIndex();
    }

// Function to get the wind value at a specific point using nearest neighbor search
    WindVal Array3D::getWindValue(double x, double y, double z) const {
        // Prepare a query point
        double queryPoint[3] = {x, y, z};

        // Variables for nearest neighbor search result
        size_t nearestIdx;
        double outDistSqr;

        // Perform kd-tree search
        nanoflann::KNNResultSet<double> resultSet(1); // Looking for 1 nearest neighbor
        resultSet.init(&nearestIdx, &outDistSqr);
        kdTree->findNeighbors(resultSet, &queryPoint[0], nanoflann::SearchParameters(10));

        // Return the corresponding wind value for the nearest point
        return windValues[nearestIdx];
    }

// Implementation of the required methods for nanoflann compatibility

// Function to return the number of data points in the kd-tree
    size_t Array3D::kdtree_get_point_count() const {
        return positions.size();
    }

// Function to get a specific dimension of a point for kd-tree search
    double Array3D::kdtree_get_pt(const size_t idx, int dim) const {
        return positions[idx][dim];
    }

// Optional function to compute the bounding box of the points (not mandatory for kd-tree search)
    template <class BBOX>
    bool Array3D::kdtree_get_bbox(BBOX& /*bb*/) const {
        return false;
    }

} // namespace WindDataProcessor
