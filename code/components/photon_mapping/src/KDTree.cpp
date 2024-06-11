#include "KDTree.hpp"

KDTree::KDTree() : root(nullptr) {}

KDTree::~KDTree() {
    deleteKDTree(root);
}

void KDTree::build(const std::vector<Photon> &photons) {
    std::vector<Photon> photonCopy = photons;
    root = buildKDTree(photonCopy, 0);
}

std::vector<Photon> KDTree::findNearbyPhotons(const glm::vec3 &position, float radius, int maxPhotons) const {
    std::priority_queue<std::pair<float, Photon>> pq;
    searchKDTree(root, position, radius, maxPhotons, pq, 0);

    std::vector<Photon> result;
    while (!pq.empty()) {
        result.push_back(pq.top().second);
        pq.pop();
    }
    return result;
}

KDNode *KDTree::buildKDTree(std::vector<Photon> &photons, int depth) {
    if (photons.empty()) return nullptr;

    int axis = depth % 3;
    std::sort(photons.begin(), photons.end(), [axis](const Photon &a, const Photon &b) {
        return a.position[axis] < b.position[axis];
        });

    int median = photons.size() / 2;
    KDNode *node = new KDNode();
    node->photon = photons[median];

    std::vector<Photon> leftPhotons(photons.begin(), photons.begin() + median);
    std::vector<Photon> rightPhotons(photons.begin() + median + 1, photons.end());

    node->left = buildKDTree(leftPhotons, depth + 1);
    node->right = buildKDTree(rightPhotons, depth + 1);

    return node;
}

void KDTree::searchKDTree(KDNode *node, const glm::vec3 &position, float radius, int maxPhotons, std::priority_queue<std::pair<float, Photon>> &pq, int depth) const {
    if (!node) return;

    float dist = glm::distance(position, node->photon.position);
    if (dist < radius) {
        pq.push({ dist, node->photon });
        if (pq.size() > maxPhotons) {
            pq.pop();
        }
    }

    int axis = depth % 3;
    if (position[axis] - radius < node->photon.position[axis]) {
        searchKDTree(node->left, position, radius, maxPhotons, pq, depth + 1);
    }
    if (position[axis] + radius > node->photon.position[axis]) {
        searchKDTree(node->right, position, radius, maxPhotons, pq, depth + 1);
    }
}

void KDTree::deleteKDTree(KDNode *node) {
    if (node) {
        deleteKDTree(node->left);
        deleteKDTree(node->right);
        delete node;
    }
}
