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

namespace WindDataProcessor {

// 3D array implementation

    Array3D::Array3D(const std::string& csvFile, const std::string& binFile) {
        this->csvFileName = csvFile;
        this->binFileName = binFile;

        determineDimensions();

        xDim = xMax - xMin + 1;
        yDim = yMax - yMin + 1;
        zDim = zMax - zMin + 1;
    }

    void Array3D::determineDimensions() {
        xMin = yMin = zMin = std::numeric_limits<int>::max();
        xMax = yMax = zMax = std::numeric_limits<int>::min();

        std::ifstream file(csvFileName);
        if (!file) {
            throw std::runtime_error("Failed to open CSV file: " + csvFileName);
        }

        std::string line;
        std::getline(file, line); // Skip the header line

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;

            int x, y, z;

            std::getline(ss, cell, ',');
            x = std::stoi(cell);

            std::getline(ss, cell, ',');
            y = std::stoi(cell);

            std::getline(ss, cell, ',');
            z = std::stoi(cell);

            xMin = std::min(xMin, x);
            xMax = std::max(xMax, x);
            yMin = std::min(yMin, y);
            yMax = std::max(yMax, y);
            zMin = std::min(zMin, z);
            zMax = std::max(zMax, z);
        }

        file.close();
    }

    void Array3D::loadData() {
        data.resize(zDim, std::vector<std::vector<WindVal>>(yDim, std::vector<WindVal>(xDim)));

        std::ifstream binFileStream(binFileName, std::ios::binary);
        if (binFileStream) {
            binFileStream.close();
            loadBinaryFile(binFileName);
        } else {
            std::ifstream file(csvFileName);
            if (!file) {
                std::cout << "Failed to open file: " << csvFileName << std::endl;
                return;
            }

            std::string line;
            std::getline(file, line); // Skip the header line

            while (std::getline(file, line)) {
                std::stringstream ss(line);
                std::string cell;

                int x, y, z;
                WindVal val;

                std::getline(ss, cell, ',');
                x = std::stoi(cell);

                std::getline(ss, cell, ',');
                y = std::stoi(cell);

                std::getline(ss, cell, ',');
                z = std::stoi(cell);

                std::getline(ss, cell, ',');
                val.u = (cell.empty()) ? 0.0f : std::stof(cell);

                std::getline(ss, cell, ',');
                val.v = (cell.empty()) ? 0.0f : std::stof(cell);

                std::getline(ss, cell);
                val.w = (cell.empty()) ? 0.0f : std::stof(cell);

                data[z - zMin][y - yMin][x - xMin] = val;
            }

            file.close();
            saveToFile("../../../Tools/sitl_gazebo/include/data_processor/3darr.bin");
        }
    }

    void Array3D::clearData() {
        data.clear();
    }

    WindVal& Array3D::operator()(int x, int y, int z) {
        return data[z - zMin][y - yMin][x - xMin];
    }

    const WindVal& Array3D::operator()(int x, int y, int z) const {
        return data[z - zMin][y - yMin][x - xMin];
    }

    std::vector<int> Array3D::getSize() const {
        return {zDim, yDim, xDim};
    }

    void Array3D::saveToFile(const std::string& filename) const {
        std::ofstream file(filename, std::ios::binary);
        if (!file) {
            std::cout << "Failed to open file for writing: " << filename << std::endl;
            return;
        }

        file.write(reinterpret_cast<const char*>(&zDim), sizeof(int));
        file.write(reinterpret_cast<const char*>(&yDim), sizeof(int));
        file.write(reinterpret_cast<const char*>(&xDim), sizeof(int));

        for (int z = 0; z < zDim; ++z) {
            for (int y = 0; y < yDim; ++y) {
                file.write(reinterpret_cast<const char*>(data[z][y].data()), xDim * sizeof(WindVal));
            }
        }

        file.close();
    }

    void Array3D::loadBinaryFile(const std::string& filename) {
        std::ifstream file(filename, std::ios::binary);
        if (!file) {
            std::cout << "Failed to open file for reading: " << filename << std::endl;
            return;
        }

        file.read(reinterpret_cast<char*>(&zDim), sizeof(int));
        file.read(reinterpret_cast<char*>(&yDim), sizeof(int));
        file.read(reinterpret_cast<char*>(&xDim), sizeof(int));

        data.resize(zDim, std::vector<std::vector<WindVal>>(yDim, std::vector<WindVal>(xDim)));

        for (int z = 0; z < zDim; ++z) {
            for (int y = 0; y < yDim; ++y) {
                file.read(reinterpret_cast<char*>(data[z][y].data()), xDim * sizeof(WindVal));
            }
        }

        file.close();
    }

    WindVal Array3D::getWindValue(int x, int y, int z) const {
        if (z < zMin || z > zMax || y < yMin || y > yMax || x < xMin || x > xMax) {
            return {0.0f, 0.0f, 0.0f};
        }

        return data[z - zMin][y - yMin][x - xMin];
    }

    void Array3D::computePointsSerial3DArray(const std::vector<Position>& dronePositions, int side) {
        loadData();

        int halfSide = side / 2;
        int cubeXDim = side;
        int cubeYDim = side;
        int cubeZDim = side;

        cubePositions3DArray.resize(dronePositions.size(), std::vector<std::vector<std::vector<WindVal>>>(
                cubeZDim, std::vector<std::vector<WindVal>>(cubeYDim, std::vector<WindVal>(cubeXDim))));

        dronePosOffsets = dronePositions;
        cubeSide = side;

        for (size_t droneIndex = 0; droneIndex < dronePositions.size(); ++droneIndex) {
            const Position& dronePos = dronePositions[droneIndex];

            for (int z = -halfSide; z <= halfSide; ++z) {
                for (int y = -halfSide; y <= halfSide; ++y) {
                    for (int x = -halfSide; x <= halfSide; ++x) {
                        int cubeX = x + halfSide;
                        int cubeY = y + halfSide;
                        int cubeZ = z + halfSide;

                        WindVal value = getWindValue(dronePos.z + z, dronePos.y + y, dronePos.x + x);

                        cubePositions3DArray[droneIndex][cubeZ][cubeY][cubeX] = value;
                    }
                }
            }
        }

        clearData();
    }

    WindVal Array3D::getCubeWindValue(int droneIndex, int x, int y, int z) const {
        if (droneIndex < 0 || droneIndex >= static_cast<int>(dronePosOffsets.size())) {
            return {0.0f, 0.0f, 0.0f};
        }

        const Position& dronePosOffset = dronePosOffsets[droneIndex];
        int halfSide = cubeSide / 2;
        int cubeX = x - dronePosOffset.x + halfSide;
        int cubeY = y - dronePosOffset.y + halfSide;
        int cubeZ = z - dronePosOffset.z + halfSide;

        if (cubeX < 0 || cubeX >= cubeSide || cubeY < 0 || cubeY >= cubeSide || cubeZ < 0 || cubeZ >= cubeSide) {
            return {0.0f, 0.0f, 0.0f};
        }

        return cubePositions3DArray[droneIndex][cubeZ][cubeY][cubeX];
    }

    size_t Array3D::calculateCubePositions3DArrayMemoryUsage() {
        size_t totalMemory = 0;

        if (cubePositions3DArray.empty()) {
            return totalMemory;
        }

        int numDronePositions = cubePositions3DArray.size();
        int zDim = cubePositions3DArray[0].size();
        int yDim = zDim > 0 ? cubePositions3DArray[0][0].size() : 0;
        int xDim = yDim > 0 ? cubePositions3DArray[0][0][0].size() : 0;

        size_t sizeOfWindVal = sizeof(WindVal);

        size_t numElements = numDronePositions * zDim * yDim * xDim;

        totalMemory = numElements * sizeOfWindVal;

        return totalMemory;
    }

    void Array3D::printCubePositions3DArrayMemoryUsage() {
        size_t totalMemory = calculateCubePositions3DArrayMemoryUsage();
        std::cout << "Total memory used by cubePositions3DArray: " << totalMemory << " bytes" << std::endl;
        std::cout << "Total memory used by cubePositions3DArray: " << static_cast<double>(totalMemory) / (1024 * 1024) << " MB" << std::endl;
    }

    void Array3D::printMemoryUsage() const {
        size_t totalMemory = 0;

        int zDim = data.size();
        if (zDim > 0) {
            int yDim = data[0].size();
            if (yDim > 0) {
                int xDim = data[0][0].size();
                totalMemory = zDim * yDim * xDim * sizeof(WindVal);
            }
        }

        std::cout << "Total memory used by Array3D data: " << totalMemory << " bytes" << std::endl;
        std::cout << "Total memory used by Array3D data: " << static_cast<double>(totalMemory) / (1024 * 1024) << " MB" << std::endl;
    }

} // namespace WindDataProcessor
