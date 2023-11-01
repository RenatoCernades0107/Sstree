#include "SStree.h"

void SsTree::insert(const Point &point, const std::string &path) {
    if (root == nullptr) {
        root = new SsLeaf(D);
        root->centroid = point;
        dynamic_cast<SsLeaf*>(root)->points.push_back(point);
        dynamic_cast<SsLeaf*>(root)->paths.push_back(path);
        return;
    }

    std::pair<SsNode*, SsNode*> nodes = _insert(point, path, this->root);

    if (nodes.second != nullptr) {
        this->root = new SsInnerNode(D);
        nodes.first->parent = root;
        nodes.second->parent = root;
        dynamic_cast<SsInnerNode*>(root)->children.push_back(nodes.first);
        dynamic_cast<SsInnerNode*>(root)->children.push_back(nodes.second);
        this->root->parent = nullptr;
        this->root->updateBoundingEnvelopeImproved();
    }
}

std::pair<SsNode *, SsNode *> SsTree::_insert(const Point &point, const std::string &path, SsNode *node) {
    if (node->isLeaf()) {
        auto* leafNode = dynamic_cast<SsLeaf*>(node);
        for (const Point& p : leafNode->points) {
            if (p == point) {
                return std::make_pair(nullptr, nullptr);
            }
        }
        leafNode->points.push_back(point);
        leafNode->paths.push_back(path);
        leafNode->updateBoundingEnvelopeImproved();
        if (leafNode->points.size() <= Settings::M) {
            return std::make_pair(nullptr, nullptr);
        }
    }
    else {
        auto* innerNode = dynamic_cast<SsInnerNode*>(node);
        auto* closestChild = innerNode->findClosestChild(point);
        std::pair<SsNode*, SsNode*> node_pair = _insert(point, path, closestChild);
        if (node_pair.second == nullptr) {
            innerNode->updateBoundingEnvelopeImproved();
            return std::make_pair(nullptr, nullptr);
        }
        else {
            node_pair.first->parent = innerNode;
            node_pair.second->parent = innerNode;
            innerNode->children.push_back(node_pair.second);
            innerNode->updateBoundingEnvelopeImproved();
            if (innerNode->children.size() <= Settings::M) {
                return std::make_pair(nullptr, nullptr);
            }
        }
    }
    return node->split();
}

void SsTree::insert(const Point& point) {
    if (root == nullptr) {
        root = new SsLeaf(D);
        root->centroid = point;
        root->insert(point);
        return;
    }

    std::pair<SsNode*, SsNode*> nodes = _insert(point, this->root);

    if (nodes.second != nullptr) {
        this->root = new SsInnerNode(D);
        nodes.first->parent = root;
        nodes.second->parent = root;
        dynamic_cast<SsInnerNode*>(root)->children.push_back(nodes.first);
        dynamic_cast<SsInnerNode*>(root)->children.push_back(nodes.second);
        this->root->parent = nullptr;
        this->root->updateBoundingEnvelopeImproved();
    }
    std::cout << "radio" << this->root->radius << std::endl;
}

std::pair<SsNode *, SsNode *> SsTree::_insert(const Point& point, SsNode* node) {
    if (node->isLeaf()) {
        auto* leafNode = dynamic_cast<SsLeaf*>(node);
        for (const Point& p : leafNode->points) {
            if (p == point) {
                return std::make_pair(nullptr, nullptr);
            }
        }
        leafNode->points.push_back(point);
        leafNode->updateBoundingEnvelopeImproved();
        if (leafNode->points.size() <= Settings::M) {
            return std::make_pair(nullptr, nullptr);
        }
    }
    else {
        auto* innerNode = dynamic_cast<SsInnerNode*>(node);
        auto* closestChild = innerNode->findClosestChild(point);
        std::pair<SsNode*, SsNode*> node_pair = _insert(point, closestChild);
        if (node_pair.second == nullptr) {
            innerNode->updateBoundingEnvelopeImproved();
            return std::make_pair(nullptr, nullptr);
        }
        else {
            node_pair.first->parent = innerNode;
            node_pair.second->parent = innerNode;
            innerNode->children.push_back(node_pair.second);
            innerNode->updateBoundingEnvelopeImproved();
            if (innerNode->children.size() <= Settings::M) {
                return std::make_pair(nullptr, nullptr);
            }
        }
    }
    return node->split();
}


std::pair<SsNode*, SsNode*> SsLeaf::insert(const Point &point) {
    this->points.push_back(point);
    return std::make_pair(nullptr, nullptr);
}

std::pair<SsNode*, SsNode*> SsInnerNode::insert(const Point &point) {
    return std::make_pair(nullptr, nullptr);
}

SsNode *SsInnerNode::findClosestChild(const Point &target) const {
    Safe<float> mindist = Safe<float>::max_value();
    SsNode* result_node;
    for (auto& node : this->children) {
        if (distance(node->centroid, target) < mindist) {
            mindist = distance(node->centroid, target);
            result_node = node;
        }
    }
    return result_node;
}


size_t SsNode::directionOfMaxVariance() const {
    NType maxVariance = 0;
    size_t directionIndex = 0;

    for (int i = 0; i < this->centroid.dim(); ++i) {
        NType variance = varianceAlongDirection(this->getEntriesCentroids(), i);
        if (variance > maxVariance) {
            maxVariance = variance;
            directionIndex = i;
        }
    }
    return directionIndex;
}

void SsInnerNode::updateBoundingEnvelope() {
    auto points = this->getEntriesCentroids();
    this->centroid = points[0];
    for (int i = 0; i < points[0].dim(); ++i) {
        // Compute mean
        NType mean = 0;
        for (auto& p : points) {
            mean += p[i];
        }
        mean = mean / static_cast<float>(points.size());
        this->centroid[i] = mean;
    }

    NType maxDist = 0;
    NType minDist = NType::max_value();
    for (const auto& child : children) {
        NType d = distance(this->centroid, child->centroid) + child->radius;
        if (d > maxDist) {
            maxDist = d;
        }
        if (d < minDist) {
            minDist = d;
        }
    }
    this->minRadius = minDist;
    this->radius = maxDist;
}


void SsLeaf::updateBoundingEnvelope() {
    auto entries = this->getEntriesCentroids();
    this->centroid = entries[0];
    for (int i = 0; i < entries[0].dim(); ++i) {
        // Compute mean
        NType mean = 0;
        for (auto& p : entries) {
            mean += p[i];
        }
        mean = mean / static_cast<NType>(entries.size());
        this->centroid[i] = mean;
    }

    NType maxDist = 0;
    NType minDist = NType::max_value();
    for (const auto& point : points) {
        NType d = distance(this->centroid, point);
        if (d > maxDist) {
            maxDist = d;
        }
        if (d < minDist) {
            minDist = d;
        }
    }
    this->minRadius = minDist;
    this->radius = maxDist;
}


std::pair<SsNode *, SsNode *> SsInnerNode::split() {
    size_t splitIndex = this->findSplitIndex();
    auto* newNode = new SsInnerNode(D);

    std::vector<SsNode*> children1;
    std::vector<SsNode*> children2;

    for (size_t j = 0; j < splitIndex; ++j) {
        children1.push_back(this->children[j]);
    }

    for (size_t j = splitIndex; j < Settings::M + 1; ++j) {
        children2.push_back(this->children[j]);
    }

    this->children = children1;
    newNode->children = children2;

    this->updateBoundingEnvelopeImproved();
    newNode->updateBoundingEnvelopeImproved();

    return std::make_pair(this, newNode);
}

std::pair<SsNode *, SsNode *> SsLeaf::split() {
    size_t splitIndex = this->findSplitIndex();
    auto* newNode = new SsLeaf(D);

    std::vector<Point> points1;
    std::vector<Point> points2;

    std::vector<std::string> path1;
    std::vector<std::string> path2;

    for (size_t j = 0; j < splitIndex; ++j) {
        points1.push_back(this->points[j]);
        if (!paths.empty()) {
            path1.push_back(this->paths[j]);
        }
    }

    for (size_t j = splitIndex; j < Settings::M + 1; ++j) {
        points2.push_back(this->points[j]);
        if (!paths.empty()) {
            path2.push_back(this->paths[j]);
        }
    }

    this->points = points1;
    newNode->points = points2;

    this->paths = path1;
    newNode->paths = path2;

    this->updateBoundingEnvelopeImproved();
    newNode->updateBoundingEnvelopeImproved();

    return std::make_pair(this, newNode);
}

inline auto calcMeanPointLeafNode = [](const std::vector<Point>& pts, Point& centroid){
    if (pts.empty()) return;
    for (int i = 0; i < pts[0].dim(); ++i) {
        // Compute mean for i dimension
        NType mean = 0;
        for (auto& p : pts) {
            mean += p[i];
        }
        mean = mean / static_cast<float>(pts.size());
        centroid[i] = mean;
    }
};


inline auto calcMeanLeafNode = [](const std::vector<std::pair<Point, std::string>>& pts, Point& centroid){
    if (pts.empty()) return;
    for (int i = 0; i < pts[0].first.dim(); ++i) {
        // Compute mean for i dimension
        NType mean = 0;
        for (auto& p : pts) {
            mean += p.first[i];
        }
        mean = mean / static_cast<float>(pts.size());
        centroid[i] = mean;
    }
};

inline auto calcMeanInnerNode = [](const std::vector<SsNode*>& nodes, Point& centroid){
    if (nodes.empty()) return;
    for (int i = 0; i < nodes[0]->centroid.dim(); ++i) {
        // Compute mean for i dimension
        NType mean = 0;
        for (const auto &node: nodes) {
            mean += node->centroid[i];
        }
        mean = mean / static_cast<float>(nodes.size());
        centroid[i] = mean;
    }
};

inline auto updateRadiusLeafNode = [](const std::vector<Point>& points, const Point& centroid, NType& radius) {
    NType maxDist = 0;
    for (const auto& point : points) {
        NType d = distance(centroid, point);
        if (d > maxDist) {
            maxDist = d;
        }
    }
    radius = maxDist;
};

inline auto updateRadiusInnernNode = [](const std::vector<SsNode*>& nodes, const Point& centroid, NType& radius) {
    NType maxDist = 0;
    for (const auto& node : nodes) {
        NType d = distance(centroid, node->centroid) + node->radius;
        if (d > maxDist) {
            maxDist = d;
        }
    }
    radius = maxDist;
};

inline auto shareNeareastChildWithInnerNode = [](std::vector<SsNode*>& class0, std::vector<SsNode*>& class1, const Point& centroid){
    int n_missing = Settings::m - class1.size();
    auto comp = [centroid](std::pair<SsNode*, int>& p1, std::pair<SsNode*, int>& p2){
        return distance(p1.first->centroid, centroid) + p1.first->radius > distance(p2.first->centroid, centroid) + p2.first->radius;
    };;

    std::priority_queue<std::pair<SsNode*, int>, std::vector<std::pair<SsNode*, int>>, decltype(comp) > nearestPoints(comp);
    int i = 0;
    for (const auto & node : class0) {
        if (nearestPoints.size() > n_missing) {
            if (distance(node->centroid, centroid) + node->radius < distance(nearestPoints.top().first->centroid, centroid) + nearestPoints.top().first->radius) {
                nearestPoints.pop();
                nearestPoints.push({node, i++});
            }
        } else {
            nearestPoints.push({node, i++});
        }
    }

    for (int j = 0; j < n_missing; ++j) {
        class1.push_back(nearestPoints.top().first);
        class0.erase(std::next(std::begin(class0), nearestPoints.top().second));
        nearestPoints.pop();
    }
};

inline auto shareNeareastChildWithLeaf = [](std::vector<std::pair<Point, std::string>>& class0, std::vector<std::pair<Point, std::string>>& class1, const Point& centroid){
    int n_missing = Settings::m - class1.size();
    auto comp = [centroid](std::tuple<Point, int, std::string>& p1, std::tuple<Point, int, std::string>& p2){
        return distance(std::get<0>(p1), centroid) > distance(std::get<0>(p2), centroid);
    };;

    std::priority_queue<std::tuple<Point, int, std::string>, std::vector<std::tuple<Point, int, std::string>>, decltype(comp)> nearestPoints(comp);
    int i = 0;
    for (auto& point : class0) {
        if (nearestPoints.size() > n_missing) {
            if (distance(point.first, centroid) < distance(std::get<0>(nearestPoints.top()), centroid)) {
                nearestPoints.pop();
                nearestPoints.push(std::make_tuple(point.first, i++, point.second));
            }
        } else {
            nearestPoints.push(std::make_tuple(point.first, i++, point.second));
        }
    }

    for (int j = 0; j < n_missing; ++j){
        auto tuple = nearestPoints.top();
        class1.push_back(std::make_pair(std::get<0>(tuple), std::get<2>(tuple)));
        class0.erase(std::next(std::begin(class0), std::get<1>(tuple)));
        nearestPoints.pop();
    }
};

const NType ERROR = 0.1;

const int MAX_ITER = 2000; // 100 -> 15.4487

const NType TOL = 1e-2;

auto getFurtherPoint = [](const std::vector<Point>& points, const Point& centroid){
    Point result;
    NType minDist = NType::max_value();
    for (const auto& point : points) {
        if (distance(point, centroid) < minDist) {
            result = point;
        }
    }
    return result;
};

auto getFurtherPointInnerNode = [](const std::vector<SsNode*>& children, const Point& centroid){
    Point result;
    NType minDist = NType::max_value();
    for (const SsNode* child : children) {
        if (distance(child->centroid, centroid) + child->radius < minDist)  {
            result = child->centroid;
        }
    }
    return result;
};


void SsInnerNode::updateBoundingEnvelopeImproved() {

    Point c = this->children[0]->centroid;
    calcMeanPointLeafNode(this->getEntriesCentroids(), c);

    int i = 0;
    while (true) {
        Point f = getFurtherPointInnerNode(this->children, c);
        Point c_next = c*NType(i)/(NType(i)+1) + f*NType(1)/(NType(i)+1);
        if (distance(c_next, c) < TOL || i > MAX_ITER) {
            break;
        }
        c = c_next;
        ++i;
    }

    this->centroid = c;
    updateRadiusInnernNode(this->children, c, this->radius);

//    std::vector<SsNode*> entries = this->children;
//    size_t n = entries.size();
//    std::random_device rd;
//    std::uniform_int_distribution<int> dis(0, n-1);
//
//    size_t splitIndex = this->findSplitIndex();
//    int index = dis(rd);
//    SsNode* seed1 = entries[index]; SsNode* seed2 = entries[(index+1)%n];
//
//    Point V = seed2->centroid - seed1->centroid, U = V/V.norm();
//    Point P = seed1->centroid - U*seed1->radius, Q = seed2->centroid + U*seed2->radius;
//    Point centro = (P + Q) / 2;
//
//    std::vector<SsNode*> current_points = {seed1, seed2};
//    NType parent_radius = distance(centro, seed1->centroid) + seed1->radius;
//
//    std::pair<Point, NType> min_sphere = {centro, parent_radius};
//
//    for (const auto& node : entries) {
//        if (node == seed1 || node == seed2) continue;
//        NType dist = distance(centro, node->centroid) + node->radius;
//        current_points.push_back(node);
//
//        if (dist <= parent_radius) {
//            continue;
//        }
//
//        NType step = dist/MAX_ITER;
//        Point vec = node->centroid - centro;
//        Point u = vec/vec.norm(); // Vector unitario de centro a point
//
//        NType current_radius = parent_radius;
//        Point current_center = centro;
//
//        std::pair<Point, NType> tmp_sphere = {centro, current_radius};
//        updateRadiusInnernNode(current_points, current_center, tmp_sphere.second);
//        current_radius = tmp_sphere.second;
//
//        for (int i = 0; i < MAX_ITER; ++i) {
//            if (current_radius < tmp_sphere.second) {
//                tmp_sphere.first = current_center;
//                tmp_sphere.second = current_radius;
//            }
//            current_center += u*step;
//            updateRadiusInnernNode(current_points, current_center, current_radius);
//        }
//        min_sphere.first = tmp_sphere.first;
//        min_sphere.second = tmp_sphere.second;
//    }
//
//    this->centroid = min_sphere.first;
//    this->radius = min_sphere.second;
}

void SsLeaf::updateBoundingEnvelopeImproved() {
    Point c = this->points[0];
    calcMeanPointLeafNode(this->points, c);

    int i = 1;
    while (true) {
        Point f = getFurtherPoint(this->points, c);
        Point c_next = c*NType(i/(i+1)) + f*NType(1/(i+1));
        if (distance(c_next, c) < TOL || i > MAX_ITER) {
            break;
        }
        c = c_next;
        ++i;
    }

    this->centroid = c;
    updateRadiusLeafNode(points, c, this->radius);


//    size_t n = this->points.size();
//    std::random_device rd;
//    std::uniform_int_distribution<int> dis(0, n-1);
//
//    int index = dis(rd);
//    Point seed1 = this->points[index], seed2 = this->points[(index+1)%n];
//    Point centro = (seed1 + seed2) / 2;
//
//    std::vector<Point> current_points = {seed1, seed2};
//    NType parent_radius = distance(centro, seed1);
//
//    std::pair<Point, NType> min_sphere = {centro, parent_radius};
//
//    for (const auto& point : this->points) {
//        if (point == seed1 || point == seed2) continue;
//
//        NType dist = distance(centro, point);
//        current_points.push_back(point);
//
//        if (dist <= parent_radius) {
//            continue;
//        }
//
//        NType step = dist/MAX_ITER;
//        Point vec = point - centro;
//        Point u = vec/vec.norm(); // Vector unitario de centro a point
//
//        NType current_radius = parent_radius;
//        Point current_center = centro;
//
//        std::pair<Point, NType> tmp_sphere = {centro, current_radius};
//        updateRadiusLeafNode(current_points, current_center, tmp_sphere.second);
//
//        current_radius = tmp_sphere.second;
//
//        for (int i = 0; i < MAX_ITER; ++i) {
//            if (current_radius < tmp_sphere.second) {
//                tmp_sphere.first = current_center;
//                tmp_sphere.second = current_radius;
//            }
//            current_center += u*step;
//            updateRadiusLeafNode(current_points, current_center, current_radius);
//        }
//        min_sphere.first = tmp_sphere.first;
//        min_sphere.second = tmp_sphere.second;
//    }
//    this->centroid = min_sphere.first;
//    this->radius = min_sphere.second;
}


std::pair<SsNode *, SsNode *> SsInnerNode::kMeansSplit() {
    auto* newNode = new SsInnerNode(D);

    size_t n = this->children.size();

    size_t coordinateIndex = this->directionOfMaxVariance();
    this->sortEntriesByCoordinate(coordinateIndex);

    std::vector<Point> centroids = {this->children[0]->centroid, this->children[n-1]->centroid};
    std::vector<SsNode*> class0, class1;
    std::vector<Point> prevCentroids(2);

    int iter = 0;
    while (true) {
        for (const auto& node : this->children) {
            NType distClass0 = max(NType(0), distance(node->centroid, centroids[0]));
            NType distClass1 = max(NType(0),distance(node->centroid, centroids[1]));

            if (distClass0 < distClass1) {
                class0.push_back(node);
            } else if (distClass0 > distClass1) {
                class1.push_back(node);
            } else {
                if (class0.size() > class1.size()) class1.push_back(node);
                else { class0.push_back(node); }
            }
        }

        if (class0.size() < Settings::m) {
            shareNeareastChildWithInnerNode(class1, class0, centroids[0]);
        }
        else if (class1.size() < Settings::m) {
            shareNeareastChildWithInnerNode(class0, class1, centroids[1]);
        }

        prevCentroids[0] = centroids[0];
        prevCentroids[1] = centroids[1];

        calcMeanInnerNode(class0, centroids[0]);
        calcMeanInnerNode(class1, centroids[1]);

        if (iter++ >= MAX_ITER || ( distance(prevCentroids[0], centroids[0]) < ERROR && distance(prevCentroids[1], centroids[1]) < ERROR )) {
            break;
        }

        class0.clear();
        class1.clear();
    }

    this->children = class0;
    newNode->children = class1;

    this->updateBoundingEnvelopeImproved();
    newNode->updateBoundingEnvelopeImproved();

    return std::make_pair(this, newNode);
}

std::pair<SsNode *, SsNode *> SsLeaf::kMeansSplit() {
    auto* newNode = new SsLeaf();

    size_t n = this->points.size();

    size_t coordinateIndex = this->directionOfMaxVariance();
    this->sortEntriesByCoordinate(coordinateIndex);

    std::vector<Point> centroids = {points[0], points[n-1]};
    std::vector<std::pair<Point, std::string>> class0, class1;
    std::vector<Point> prevCentroids(2);

    int iter = 0;
    while (true) {
        int index = 0;
        for (const auto& point : this->points) {
            NType distClass0 = max(NType(0), distance(point, centroids[0]));
            NType distClass1 = max(NType(0),distance(point, centroids[1]));
            if (distClass0 < distClass1) {
                class0.push_back(std::make_pair(point, paths[index]));
            } else if (distClass0 > distClass1) {
                class1.push_back(std::make_pair(point, paths[index]));
            } else {
                if (class0.size() > class1.size()) class1.push_back(std::make_pair(point, paths[index]));
                else { class0.push_back(std::make_pair(point, paths[index])); }
            }
            index++;
        }

        if (class0.size() < Settings::m) {
            shareNeareastChildWithLeaf(class1, class0, centroids[0]);
        }
        else if (class1.size() < Settings::m) {
            shareNeareastChildWithLeaf(class0, class1, centroids[1]);
        }

        prevCentroids[0] = centroids[0];
        prevCentroids[1] = centroids[1];

        calcMeanLeafNode(class0, centroids[0]);
        calcMeanLeafNode(class1, centroids[1]);

        if (iter++ >= MAX_ITER || ( distance(prevCentroids[0], centroids[0]) < ERROR && distance(prevCentroids[1], centroids[1]) < ERROR )) {
            break;
        }

        class0.clear();
        class1.clear();
    }

    this->points.clear();
    this->paths.clear();
    for (int i = 0; i < class0.size(); ++i) {
        this->points.push_back(class0[i].first);
        this->paths.push_back(class0[i].second);
    }

    newNode->points.clear();
    newNode->paths.clear();
    for (int i = 0; i < class1.size(); ++i) {
        newNode->points.push_back(class1[i].first);
        newNode->paths.push_back(class1[i].second);
    }

    this->updateBoundingEnvelopeImproved();
    newNode->updateBoundingEnvelopeImproved();

    return std::make_pair(this, newNode);
}

size_t SsNode::findSplitIndex() {
    size_t coordinateIndex = this->directionOfMaxVariance();
    this->sortEntriesByCoordinate(coordinateIndex);
    return minVarianceSplit(coordinateIndex);
}

std::vector<Point> SsInnerNode::getEntriesCentroids() const {
    std::vector<Point> result;
    for (auto& child : children) {
        result.push_back(child->centroid);
    }
    return result;
}

std::vector<Point> SsLeaf::getEntriesCentroids() const {
    return points;
}

void SsLeaf::sortEntriesByCoordinate(size_t coordinateIndex) {
    std::vector<std::pair<Point, std::string>> tmp;
    for (int i = 0; i < points.size(); ++i) {
        if (paths.empty()) {
            tmp.push_back(std::make_pair(points[i], ""));
        } else {
            tmp.push_back(std::make_pair(points[i], paths[i]));
        }
    }

    std::sort(tmp.begin(), tmp.end(), [coordinateIndex](std::pair<Point, std::string>& p1, std::pair<Point, std::string>& p2) {
        return p1.first[coordinateIndex] < p2.first[coordinateIndex];
    });

    for (int i = 0; i < points.size(); ++i) {
        points[i] = tmp[i].first;
        if (!paths.empty()) {
            paths[i] = tmp[i].second;
        }
    }
}


void SsInnerNode::sortEntriesByCoordinate(size_t coordinateIndex) {
    std::sort(children.begin(), children.end(), [coordinateIndex](SsNode* p1, SsNode* p2){
        return p1->centroid[coordinateIndex] < p2->centroid[coordinateIndex];
    });
}

NType SsNode::calculateMean(const std::vector<Point> &centroids, size_t direction) const {
    NType mean = 0;
    for (const auto& point : centroids) {
        mean += point[direction];
    }
    return mean / static_cast<float>(centroids.size());
}


NType SsNode::varianceAlongDirection(const std::vector<Point> &centroids, size_t direction) const {
    NType variance = 0;
    NType mean = calculateMean(centroids, direction);
    for (const auto& point : centroids) {
        variance += pow(point[direction] - mean, 2);
    }
    return variance / static_cast<float>(centroids.size());
}

size_t SsNode::minVarianceSplit(size_t coordinateIndex) {
    NType minVariance = NType::max_value();
    size_t splitIndex = Settings::m;
    for (int i = Settings::m; i < this->getEntriesCentroids().size() - Settings::m; ++i) {

        std::vector<Point> centroids1;
        std::vector<Point> centroids2;

        for (int j = 0; j < i; ++j) {
            centroids1.push_back(this->getEntriesCentroids()[j]);
        }

        for (int j = i; j < Settings::M + 1; ++j) {
            centroids2.push_back(this->getEntriesCentroids()[j]);
        }

        NType variance1 = varianceAlongDirection(centroids1, coordinateIndex);
        NType variance2 = varianceAlongDirection(centroids2, coordinateIndex);

        if (variance1 + variance2 < minVariance) {
            minVariance = variance1 + variance2;
            splitIndex = i;
        }
    }

    return splitIndex;
}



SsNode *SsTree::search(const Point &target) {
    if (root == nullptr) return nullptr;
    return _search(root, target);
}

SsNode *SsTree::_search(SsNode *node, const Point &target) {
    if (node->isLeaf()) {
        auto* leafNode = dynamic_cast<SsLeaf*>(node);
        for(const auto& point : leafNode->points) {
            if (target == point) {
                return node;
            }
        }
    } else {
        auto* leafNode = dynamic_cast<SsInnerNode*>(node);
        SsNode* result_node;
        for (const auto& child : leafNode->children) {
            if (child->intersectsPoint(target)) {
                result_node = _search(child, target);
            }
        }
        if (result_node != nullptr) return result_node;
    }
    return nullptr;
}


// Se asume que hay más de k puntos en el árbol
std::vector<Point> SsTree::kNNQuery(const Point &center, size_t k) const {
    if (root == nullptr) return std::vector<Point>();
    std::priority_queue<std::pair<NType, Point>, std::vector<std::pair<NType, Point>>, decltype(compare)> result(compare);
    this->_kNNQuery(center, k, this->root, NType::max_value(), result);
    std::vector<Point> points;
    for (int i = 0; i < k; ++i) {
        points.push_back(result.top().second);
        result.pop();
    }
    std::vector<Point> points_inv;
    std::reverse_copy(points.begin(), points.end(), std::back_inserter(points_inv));
    return points_inv;
}



void SsTree::_kNNQuery(const Point &target, size_t k, SsNode *node, NType radius, std::priority_queue<std::pair<NType, Point>, std::vector<std::pair<NType, Point>>, decltype(compare)> &result) const {
    if (node->isLeaf()) {
        auto* leafNode = dynamic_cast<SsLeaf*>(node);
        for(auto& p : leafNode->points) {
            NType dist = std::max(NType(0), distance(target, p));
            // Regla 2
            if (radius + std::max(NType(0), dist) < std::max(NType(0), distance(target, leafNode->centroid))) {
                continue;
            }
            // Regla 4
            if (radius + std::max(NType(0), distance(target, leafNode->centroid)) < std::max(NType(0), distance(p, leafNode->centroid))) {
                continue;
            }

            if (dist < radius) {
                result.push(std::make_pair(dist, p));
                if (result.size() > k) {
                    result.pop();
                }
            }
        }
    }
    else {
        auto* innerNode = dynamic_cast<SsInnerNode*>(node);
        // Se obtiene la menor distancia hacia un nodo
        for (auto* child : innerNode->children) {
            NType dist = std::max(NType(0), distance(target, child->centroid));

            // Regla 1
            if (radius + child->radius < std::max(NType(0), distance(target, child->centroid))) {
                continue;
            }

            // Regla 3
            if (radius + std::max(NType(0), distance(target, child->centroid)) < child->minRadius) {
                continue;
            }

            if (dist + child->radius < radius) {
                _kNNQuery(target, k, child, radius, result);
            }
        }
    }
}



bool SsNode::test(bool isRoot) const {
    size_t count = 0;
    if (this->isLeaf()) {
        const SsLeaf* leaf = dynamic_cast<const SsLeaf*>(this);
        count = leaf->points.size();

        // Verificar si los puntos están dentro del radio del nodo
        for (const Point& point : leaf->points) {
            if (distance(this->centroid, point) > this->radius) {
                std::cout << "Point outside node radius detected." << std::endl;
                return false;
            }
        }
    } else {
        const SsInnerNode* inner = dynamic_cast<const SsInnerNode*>(this);
        count = inner->children.size();

        // Verificar si los centroides de los hijos están dentro del radio del nodo padre
        for (const SsNode* child : inner->children) {
            if (distance(this->centroid, child->centroid) > this->radius) {
                std::cout << "Child centroid outside parent radius detected." << std::endl;
                return false;
            }
            // Verificar recursivamente cada hijo
            if (!child->test()) {
                return false;
            }
        }
    }

    // Comprobar la validez de la cantidad de hijos/puntos
    if (!isRoot && (count < Settings::m || count > Settings::M)) {
        std::cout << "Invalid number of children/points detected." << std::endl;
        return false;
    }

    // Comprobar punteros de parentezco, salvo para el nodo raíz
    if (!isRoot && !parent) {
        std::cout << "Node without parent detected." << std::endl;
        return false;
    }

    return true;
}

void SsTree::test() const {
    bool result = root->test(true);

    if (root->parent) {
        std::cout << "Root node parent pointer is not null!" << std::endl;
        result = false;
    }

    if (result) {
        std::cout << "SS-Tree is valid!" << std::endl;
    } else {
        std::cout << "SS-Tree has issues!" << std::endl;
    }
}


void SsNode::print(size_t indent) const {
    for (size_t i = 0; i < indent; ++i) {
        std::cout << "  ";
    }

    // Imprime información del nodo.
    std::cout << "Centroid: " << centroid << ", Radius: " << radius;
    if (isLeaf()) {
        const SsLeaf* leaf = dynamic_cast<const SsLeaf*>(this);
        std::cout << ", Points: [ ";
        for (const Point& p : leaf->points) {
            std::cout << p << " ";
        }
        std::cout << "]";
    } else {
        std::cout << std::endl;
        const SsInnerNode* inner = dynamic_cast<const SsInnerNode*>(this);
        for (const SsNode* child : inner->children) {
            child->print(indent + 1); 
        }
    }
    std::cout << std::endl;
}

SsNode::SsNode(size_t d) : D(d) {}

SsNode::SsNode() {}

void SsTree::print() const {
    if (root) {
//        root->print();
        std::cout << "root: " << root->radius << std::endl;
        for (auto child : dynamic_cast<SsInnerNode*>(root)->children)
            std::cout << "child: " << child->radius << std::endl;

    } else {
        std::cout << "Empty tree." << std::endl;
    }
}



void SsLeaf::saveToStream(std::ostream &out) const {
    // Guardar centroid
    centroid.saveToFile(out, D);

    // Guardar el radio
    float radius_ = radius.getValue();
    out.write(reinterpret_cast<const char*>(&radius_), sizeof(radius_));

    // Guardar el numero de puntos
    size_t numPoints = points.size();
    out.write(reinterpret_cast<const char*>(&numPoints), sizeof(numPoints));

    // Guardar los puntos
    for (const auto& point : points) {
        point.saveToFile(out, D);
    }

    // Guardar las rutas (paths)
    size_t numPaths = paths.size();
    out.write(reinterpret_cast<const char*>(&numPaths), sizeof(numPaths));
    for (const auto& p : paths) {
        size_t pathLength = p.size();
        out.write(reinterpret_cast<const char*>(&pathLength), sizeof(pathLength));
        out.write(p.c_str(), (long) pathLength);
    }
}

void SsLeaf::loadFromStream(std::istream &in, SsNode* parent) {
    this->parent = parent;
    // Leer centroid
    centroid.readFromFile(in, D);

    // Leer radio
    float radius_ = 0;
    in.read(reinterpret_cast<char*>(&radius_), sizeof(radius_));
    this->radius = radius_;

    // Leer numero de puntos
    size_t numPoints;
    in.read(reinterpret_cast<char*>(&numPoints), sizeof(numPoints));

    // Leer puntos
    points.resize(numPoints);
    for (size_t i = 0; i < numPoints; ++i) {
        points[i].readFromFile(in, D);
    }

    // Leer rutas (paths)
    size_t numPaths;
    in.read(reinterpret_cast<char*>(&numPaths), sizeof(numPaths));
    paths.resize(numPaths);
    for (size_t i = 0; i < numPaths; ++i) {
        size_t pathLength;
        in.read(reinterpret_cast<char*>(&pathLength), sizeof(pathLength));
        char* buffer = new char[pathLength + 1];
        in.read(buffer, (long) pathLength);
        buffer[pathLength] = '\0';
        paths[i] = std::string(buffer);
        delete[] buffer;
    }
}

SsLeaf::SsLeaf(size_t d) : SsNode(d) {}

SsLeaf::SsLeaf() {}

void SsLeaf::FNDFTrav(const Point &q, size_t k, std::priority_queue<Pair, std::vector<Pair>, Comparator> &L,
                      NType &Dk) const {
}


void SsInnerNode::saveToStream(std::ostream &out) const {
    // Guardar centroid
    centroid.saveToFile(out, D);

    // Guardar el radio
    float radius_ = radius.getValue();
    out.write(reinterpret_cast<const char*>(&radius_), sizeof(radius_));

    // Guardar si apunta a nodos hoja
    bool pointsToLeafs = children[0]->isLeaf();
    out.write(reinterpret_cast<const char*>(&pointsToLeafs), sizeof(pointsToLeafs));

    // Guardar la cantidad de hijos para saber cuántos nodos leer después
    size_t numChildren = children.size();
    out.write(reinterpret_cast<const char*>(&numChildren), sizeof(numChildren));

    // Guardar los hijos
    for (const auto& child : children) {
        child->saveToStream(out);
    }
}

void SsInnerNode::loadFromStream(std::istream &in, SsNode* parent) {
    this->parent = parent;
    // Leer centroid
    centroid.readFromFile(in, D);

    // leer el valor del radio
    float radius_ = 0;
    in.read(reinterpret_cast<char*>(&radius_), sizeof(radius_));
    this->radius = radius_;

    // leer si apunta a hojas o nodos internos
    bool pointsToLeaf = false;
    in.read(reinterpret_cast<char*>(&pointsToLeaf), sizeof(pointsToLeaf));

    // leer cantidad de hijos
    size_t numChildren;
    in.read(reinterpret_cast<char*>(&numChildren), sizeof(numChildren));

    // leer hijos
    for (size_t i = 0; i < numChildren; ++i) {
        SsNode* child = pointsToLeaf ? static_cast<SsNode*>(new SsLeaf(D)) : static_cast<SsNode*>(new SsInnerNode(D));
        child->loadFromStream(in, this);
        children.push_back(child);
    }
}

SsInnerNode::SsInnerNode(size_t d) : SsNode(d) {}

SsInnerNode::SsInnerNode() {}

void SsInnerNode::FNDFTrav(const Point &q, size_t k, std::priority_queue<Pair, std::vector<Pair>, Comparator> &L,
                           NType &Dk) const {
}


void SsTree::saveToFile(const std::string &filename) const {
    std::ofstream out(filename, std::ios::binary);
    if (!out) {
        throw std::runtime_error("Cannot open file for writing");
    }

    // Guardar las dimensiones de la estructura
    out.write(reinterpret_cast<const char*>(&D), sizeof(D));

    // Guardar si el root es hija o nodo interno
    bool isLeaf = root->isLeaf();
    out.write(reinterpret_cast<const char*>(&isLeaf), sizeof(isLeaf));

    // Guardar el resto de la estructura
    root->saveToStream(out);
    out.close();
}

void SsTree::loadFromFile(const std::string &filename) {
    std::ifstream in(filename, std::ios::binary);
    if (!in) {
        throw std::runtime_error("Cannot open file for reading");
    }
    if (root) {
        delete root;
        root = nullptr;
    }

    // Aquí se asume que el primer valor determina las dimensiones
    in.read(reinterpret_cast<char*>(&D), sizeof(D));

    // El segundo valor determina si el root es hoja
    bool isLeaf;
    in.read(reinterpret_cast<char*>(&isLeaf), sizeof(isLeaf));
    if (isLeaf) {
        root = new SsLeaf(D);
    } else {
        root = new SsInnerNode(D);
    }
    root->loadFromStream(in, nullptr);
    in.close();
}