#ifndef WIND_DATA_PROCESSOR_H
#define WIND_DATA_PROCESSOR_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <mutex>

namespace WindDataProcessor {
// Structs
struct Position {
    int x, y, z;
};

struct WindVal {
    float u, v, w;
};

struct KeyHasher {
    size_t operator()(const Position& pos) const;
};

struct KeyEquals {
    bool operator()(const Position& lhs, const Position& rhs) const;
};

// Array3D class definition
class Array3D {
private:
    int cubeSide;
    int xDim, yDim, zDim;
    int xMin, xMax, yMin, yMax, zMin, zMax;
    std::string csvFileName, binFileName;
    std::vector<std::vector<std::vector<WindVal>>> data;
    std::vector<std::vector<std::vector<std::vector<WindVal>>>> cubePositions3DArray;

public:
    // data fetch operator()
    std::vector<Position> dronePosOffsets;
    WindVal& operator()(int x, int y, int z);
    const WindVal& operator()(int x, int y, int z) const;

    Array3D(const std::string& csvFile, const std::string& binFile, int xMin, int xMax, int yMin, int yMax, int zMin, int zMax);
    
    void loadData();
    void saveToFile(const std::string& filename) const;
    void loadBinaryFile(const std::string& filename);
    
    WindVal getWindValue(int x, int y, int z) const;
    void computePointsSerial3DArray(const std::vector<Position>& dronePositions, int side);
    WindVal getCubeWindValue(int droneIndex, int x, int y, int z) const;

    std::vector<int> getSize() const;
    size_t calculateCubePositions3DArrayMemoryUsage();
    void printCubePositions3DArrayMemoryUsage();
    void printMemoryUsage() const;
    void clearData();
};


} // namespace WindDataProcessor

#endif // WIND_DATA_PROCESSOR_H