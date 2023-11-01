#include <iostream>
#include <vector>
#include <random>
#include "Point.h"
#include "SStree.h"
#include <H5Cpp.h>

struct ImageData {
    Point embedding;
    std::string path;
};

std::vector<ImageData> readEmbeddingsFromHDF5(const H5std_string& FILE_NAME) {
    std::vector<ImageData> data;

    try {
        H5::H5File file(FILE_NAME, H5F_ACC_RDONLY);

        // Leer dataset "features"
        H5::DataSet featuresDataset = file.openDataSet("features");
        H5::DataSpace featuresSpace = featuresDataset.getSpace();
        const int num_embeddings = featuresSpace.getSimpleExtentNpoints();
        double* embeddingData = new double[num_embeddings];
        featuresDataset.read(embeddingData, H5::PredType::NATIVE_DOUBLE);

        // Leer dataset "paths"
        H5::DataSet pathsDataset = file.openDataSet("paths");
        H5::StrType strType = pathsDataset.getStrType();
        H5::DataSpace pathsSpace = pathsDataset.getSpace();
        const int num_paths = pathsSpace.getSimpleExtentNpoints();
        std::vector<H5std_string> paths(num_paths);
        pathsDataset.read(paths.data(), strType);

        for (int i = 0; i < num_paths; ++i) {
            Point embedding(448);
            for (int j = 0; j < 448; ++j) {
                embedding[j] = embeddingData[i * 448 + j];
            }
            data.push_back({embedding, ""});
        }

        delete[] embeddingData;
        file.close();
    } catch(H5::Exception& error) {
        std::cerr << error.getCDetailMsg() << std::endl;
    }

    return data;
}

std::vector<Point> secuential_knn(const Point& target, std::vector<ImageData>& data, int k = 1) {
    std::vector<Point> res;
    std::priority_queue<std::pair<NType, Point>, std::vector<std::pair<NType, Point>>, decltype(compare)> pq(compare);
    for (const auto& item : data) {
        pq.push(std::make_pair(-distance(item.embedding, target), item.embedding));
    }
    for (int i = 0; i < k; ++i) {
        auto par = pq.top();
        res.push_back(par.second);
        pq.pop();
    }
    return res;
}

void comp_vec(std::vector<Point>& vec1, std::vector<Point>& vec2) {
    for (const auto& p : vec1) {
        if (std::find(vec2.begin(), vec2.end(), p) == vec2.end()) {
            std::cout << "Punto no encontrado: " << p << std::endl;
            throw std::runtime_error("Test fallo");
        }
    }
}

void test_knn(SsTree& tree, std::vector<ImageData>& data) {
    for (const auto& item : data) {
        std::vector<Point> r1 = tree.kNNQuery(item.embedding, 1);
        std::vector<Point> r2 = secuential_knn(item.embedding, data);
        std::cout << "Punto KNN Sstree: " << r1[0] << " dist: " << distance(item.embedding, r1[0]) << std::endl;
        std::cout << "Punto KNN: " << r2[0] << " dist: " << distance(item.embedding, r2[0]) << std::endl;
        comp_vec(r1, r2);
    }
}

int main() {
    const H5std_string FILE_NAME("embbeding.hdf5");
    std::vector<ImageData> data = readEmbeddingsFromHDF5(FILE_NAME);

    SsTree tree;

    int i = 0;
//    for (const ImageData& item : data) {
//        item.embedding;
//        tree.insert(item.embedding, item.path);
//        if (i % 10 == 0) {
//            std::cout << i << " -> " << tree.root->radius << std::endl;
//        }
//        ++i;
//
//    }
//
    std::string filename = "embbeding.dat";
//    tree.saveToFile(filename);

    tree.loadFromFile(filename);
    tree.test();
    tree.print();

    test_knn(tree, data);

    return 0;
}
