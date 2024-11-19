#include "wind_data_processor.h"
#include <fstream>
#include <sstream>
#include <nanoflann.hpp>

namespace WindDataProcessor {

    WindDataProcessor::WindDataProcessor(const std::string &csvFile) {
        loadWindData(csvFile);
        buildKDTree();
    }

    void WindDataProcessor::loadWindData(const std::string &csvFile) {
        std::ifstream file(csvFile);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open file: " + csvFile);
        }

        std::string line;
        std::getline(file, line); // Skip the header
        while (std::getline(file, line)) {
            std::istringstream ss(line);
            Point point;
            std::string value;

            std::getline(ss, value, ',');
            point.x = std::stof(value);
            std::getline(ss, value, ',');
            point.y = std::stof(value);
            std::getline(ss, value, ',');
            point.z = std::stof(value);
            std::getline(ss, value, ',');
            point.wind.u = std::stof(value);
            std::getline(ss, value, ',');
            point.wind.v = std::stof(value);
            std::getline(ss, value, ',');
            point.wind.w = std::stof(value);

            windData.push_back(point);
        }
        file.close();
    }

    void WindDataProcessor::buildKDTree() {
        kdTreeMatrix.resize(3, windData.size());
        for (size_t i = 0; i < windData.size(); ++i) {
            kdTreeMatrix(0, i) = windData[i].x;
            kdTreeMatrix(1, i) = windData[i].y;
            kdTreeMatrix(2, i) = windData[i].z;
        }

        kdTree = std::make_unique<KDTree>(3, kdTreeMatrix, 10 /* max leaf */);
        kdTree->index_->buildIndex(); // Use index_ instead of index
    }

    WindVal WindDataProcessor::getNearestWindValue(float x, float y, float z) {
        Eigen::VectorXf query(3);
        query << x, y, z;

        std::vector<size_t> indices(1);
        std::vector<float> distances(1);

        nanoflann::KNNResultSet<float> resultSet(1);
        resultSet.init(indices.data(), distances.data());
        kdTree->index_->findNeighbors(resultSet, query.data(), nanoflann::SearchParameters(10)); // Use SearchParameters

        return windData[indices[0]].wind;
    }

} // namespace WindDataProcessor
