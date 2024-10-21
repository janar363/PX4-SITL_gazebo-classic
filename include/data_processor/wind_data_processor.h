// wind_data_processor.h
#ifndef WIND_DATA_PROCESSOR_H
#define WIND_DATA_PROCESSOR_H

#include <iostream>
#include <vector>
#include <string>
#include <nanoflann.hpp>
#include <memory>

namespace WindDataProcessor {

// Structs
    struct Position {
        double x, y, z;
    };

    struct WindVal {
        double u, v, w;
    };

// Array3D class definition
    class Array3D {
    private:
        std::string csvFileName;

        // Variables to store the min and max dimensions of the 3D space
        double minX, minY, minZ;
        double maxX, maxY, maxZ;

        // Storage for 3D points (x, y, z) and corresponding wind values (u, v, w)
        std::vector<std::array<double, 3>> positions;
        std::vector<WindVal> windValues;

        // KD-Tree-related members
        using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
                nanoflann::L2_Simple_Adaptor<double, Array3D>,
                Array3D, 3 /* dimensionality */>;

        std::unique_ptr<KDTree> kdTree;

        // Internal functions
        void determineDimensions();
        void loadData();
        void constructKDTree();

    public:
        // Constructor
        Array3D(const std::string& csvFile);

        // Function to get the wind value at a given 3D point (x, y, z)
        WindVal getWindValue(double x, double y, double z) const;

        // KD-Tree helper functions required by nanoflann
        size_t kdtree_get_point_count() const;
        double kdtree_get_pt(const size_t idx, int dim) const;
        template <class BBOX>
        bool kdtree_get_bbox(BBOX& /*bb*/) const;
    };

} // namespace WindDataProcessor

#endif // WIND_DATA_PROCESSOR_H
