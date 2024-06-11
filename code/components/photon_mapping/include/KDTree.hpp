#ifndef KDTREE_HPP
#define KDTREE_HPP

#include <vector>
#include <queue>
#include <algorithm>
#include <functional>
#include <glm/glm.hpp>

struct Photon {
    glm::vec3 position;
    glm::vec3 power;
    glm::vec3 direction;
};

struct KDNode {
    Photon photon;
    KDNode *left;
    KDNode *right;

    KDNode() : left(nullptr), right(nullptr) {}
};

class KDTree {
public:
    KDTree();
    ~KDTree();

    void build(const std::vector<Photon> &photons);
    std::vector<Photon> findNearbyPhotons(const glm::vec3 &position, float radius, int maxPhotons) const;

private:
    KDNode *buildKDTree(std::vector<Photon> &photons, int depth);
    void searchKDTree(KDNode *node, const glm::vec3 &position, float radius, int maxPhotons, std::priority_queue<std::pair<float, Photon>> &pq, int depth) const;
    void deleteKDTree(KDNode *node);

    KDNode *root;
};

#endif // KDTREE_HPP