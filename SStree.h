#ifndef SSTREE_H
#define SSTREE_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <queue>
#include <limits>
#include <fstream>
#include <random>

#include "params.h"
#include "Point.h"

struct Pair {
    Point point;
    NType distance;

    Pair(const Point& p, NType d) : point(p), distance(d) {}
};

struct Comparator {
    bool operator()(const Pair& a, const Pair& b) const {
        return a.distance < b.distance; // max-heap basado en distancia
    }
};

class SsNode {
private:
    NType varianceAlongDirection(const std::vector<Point>& centroids, size_t direction) const;
    size_t minVarianceSplit(size_t coordinateIndex);
    
public:
    explicit SsNode(size_t d);

    virtual ~SsNode() = default;

    Point centroid; 
    NType radius;

    SsNode();

    NType minRadius;
    SsNode* parent = nullptr;
    size_t D;

    NType calculateMean(const std::vector<Point>& centroids, size_t direction) const;
    virtual bool isLeaf() const = 0;
    virtual std::vector<Point> getEntriesCentroids() const = 0;
    virtual void sortEntriesByCoordinate(size_t coordinateIndex) = 0;
    virtual std::pair<SsNode*, SsNode*> split() = 0;
    virtual std::pair<SsNode*, SsNode*> kMeansSplit() = 0;

    virtual bool intersectsPoint(const Point& point) const {
        return distance(this->centroid, point) <= this->radius;
    }

    virtual void updateBoundingEnvelope() = 0;
    virtual void updateBoundingEnvelopeImproved() = 0;

    size_t directionOfMaxVariance() const;
    size_t findSplitIndex();

    virtual std::pair<SsNode*, SsNode*> insert(const Point& point) = 0;

    bool test(bool isRoot = false) const;
    void print(size_t indent = 0) const;

    virtual void FNDFTrav(const Point& q, size_t k, std::priority_queue<Pair, std::vector<Pair>, Comparator>& L, NType& Dk) const = 0;


    virtual void saveToStream(std::ostream &out) const = 0;
    virtual void loadFromStream(std::istream &in, SsNode* parent) = 0;
};

class SsInnerNode : public SsNode {
private:
    std::vector<Point> getEntriesCentroids() const override;
    void sortEntriesByCoordinate(size_t coordinateIndex) override;

public:
    SsInnerNode();

    std::pair<SsNode*, SsNode*> split() override;
    std::pair<SsNode*, SsNode*> kMeansSplit() override;

    explicit SsInnerNode(size_t d);

    std::vector<SsNode*> children;

    SsNode* findClosestChild(const Point& target) const;
    bool isLeaf() const override { return false; }
    void updateBoundingEnvelope() override;
    void updateBoundingEnvelopeImproved() override;


    std::pair<SsNode*, SsNode*> insert(const Point& point) override;

    void FNDFTrav(const Point& q, size_t k, std::priority_queue<Pair, std::vector<Pair>, Comparator>& L, NType& Dk) const override;

    virtual void saveToStream(std::ostream &out) const override;
    void loadFromStream(std::istream &in, SsNode* parent) override;

    ~SsInnerNode() {
        for (auto& child : children) {
            delete child;
        }
    }
};

class SsLeaf : public SsNode {
public:
    std::vector<std::string> paths;

    SsLeaf();

    explicit SsLeaf(size_t d);

private:
    std::vector<Point> getEntriesCentroids() const override;
    void sortEntriesByCoordinate(size_t coordinateIndex) override;

public:
    std::pair<SsNode*, SsNode*> split() override;
    std::pair<SsNode*, SsNode*> kMeansSplit() override;

    std::vector<Point> points;

    bool isLeaf() const override { return true; }
    void updateBoundingEnvelope() override;
    void updateBoundingEnvelopeImproved() override;


    std::pair<SsNode*, SsNode*> insert(const Point& point) override;

    void FNDFTrav(const Point& q, size_t k, std::priority_queue<Pair, std::vector<Pair>, Comparator>& L, NType& Dk) const override;

    void saveToStream(std::ostream &out) const override;
    void loadFromStream(std::istream &in, SsNode* parent) override;

};

inline auto compare = [](std::pair<NType, Point>& x, std::pair<NType, Point>& y){
    return x.first < y.first;
};

class SsTree {
private:
    size_t D = 448;
    SsNode* _search(SsNode* node, const Point& target);
    std::pair<SsNode *, SsNode *> _insert(const Point& point, SsNode* node);
    std::pair<SsNode *, SsNode *> _insert(const Point& point, const std::string& path, SsNode* node);
    void _kNNQuery(const Point& target, size_t k, SsNode* node, NType radius, std::priority_queue<std::pair<NType, Point>, std::vector<std::pair<NType, Point>>, decltype(compare)>& result) const;

    SsNode* searchParentLeaf(SsNode* node, const Point& target);

public:
    SsNode* root;

    SsTree() : root(nullptr) {}
    ~SsTree() {
        delete root;
    }
    
    void insert(const Point& point);
    SsNode* search(const Point& target);

    void insert(const Point& point, const std::string& path);
    void build (const std::vector<Point>& points);

    std::vector<Point> kNNQuery(const Point& center, size_t k) const;

    void print() const;
    void test() const;

    void saveToFile(const std::string &filename) const;
    void loadFromFile(const std::string &filename);
};

#endif // !SSTREE_H