#ifndef WIND_DATA_PROCESSOR_H
#define WIND_DATA_PROCESSOR_H

#include <vector>
#include <string>
#include <iostream>
#include <memory>
#include <Eigen/Core>
#include <nanoflann.hpp>

namespace WindDataProcessor {

    struct WindVal {
        float u, v, w;
    };

    struct Point {
        float x, y, z;
        WindVal wind;
    };

    class WindDataProcessor {
    private:
        std::vector<Point> windData;
        using KDTree = nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXf, 3, nanoflann::metric_L2_Simple>;
        std::unique_ptr<KDTree> kdTree;
        Eigen::MatrixXf kdTreeMatrix;

        void buildKDTree();

    public:
        WindDataProcessor(const std::string &csvFile);

        void loadWindData(const std::string &csvFile);
        WindVal getNearestWindValue(float x, float y, float z);
    };

} // namespace WindDataProcessor

#endif // WIND_DATA_PROCESSOR_H
