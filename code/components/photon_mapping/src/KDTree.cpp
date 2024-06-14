#include "KDTree.hpp"
#include <algorithm>
#include <functional>
#include <queue>
#include <stack>


namespace PhotonMapper
{
	void KDTree::build(const vector<Photon> &photons)
	{
		vector<Photon> copy_photons = photons;
		_build(copy_photons.begin(), copy_photons.end(), root);
	}

	void KDTree::_build(vector<Photon>::iterator begin, vector<Photon>::iterator end, Node*& node)
	{
		if (begin == end) return;
		node = new Node();
		if (end - begin == 1)
		{
			node->photon = *begin;
			return;
		}
		auto splitElement = _split(begin, end, node);
		node->photon = *splitElement;
		_build(begin, splitElement, node->left);
		_build(splitElement + 1, end, node->right);
	}

	vector<Photon>::iterator KDTree::_split(vector<Photon>::iterator begin, vector<Photon>::iterator end, Node*& node)
	{
		int split = chooseSplit(begin, end);
		node->split = split;
		sort(begin, end, [split](const Photon &p1, const Photon &p2) {
			if (split == 0) return p1.position.x < p2.position.x;
			if (split == 1) return p1.position.y < p2.position.y;
			if (split == 2) return p1.position.z < p2.position.z;
			});
		return begin + (end - begin) / 2;
	}

	static double variance(vector<Photon>::iterator begin, vector<Photon>::iterator end, function<double(const Photon&)> f)
	{
		double sum = 0;
		double sum2 = 0;
		for (auto it = begin; it != end; it++)
		{
			sum += f(*it);
			sum2 += f(*it) * f(*it);
		}
		double mean = sum / (end - begin);
		double mean2 = sum2 / (end - begin);
		return mean2 - mean * mean;
	}

	int KDTree::chooseSplit(vector<Photon>::iterator begin, vector<Photon>::iterator end)
	{
		double variance_x = variance(begin, end, [](const Photon &p) {return p.position.x; });
		double variance_y = variance(begin, end, [](const Photon &p) {return p.position.y; });
		double variance_z = variance(begin, end, [](const Photon &p) {return p.position.z; });
		double _max = max({ variance_x,variance_y,variance_z });
		if (_max == variance_x) return 0;
		else if (_max == variance_y) return 1;
		return 2;
	}

	void KDTree::_release(Node* node)
	{
		if (node == nullptr) return;
		_release(node->left);
		_release(node->right);
		delete node;
	}

	KDTree::~KDTree()
	{
		_release(root);
	}

	class ComparePhoton {
	private:
		Vec3 position;
	public:
		ComparePhoton(const Vec3 &position) : position(position) {}
		bool operator()(const Photon &p1, const Photon &p2) {
			return glm::distance(p1.position, position) < glm::distance(p2.position, position);
		}
	};

	static void _pq_push(priority_queue<Photon, vector<Photon>, ComparePhoton> &pq, const Photon &photon, int num)
	{
		if (pq.size() < num)
		{
			pq.push(photon);
			return;
		}
		pq.pop();
		pq.push(photon);
	}

	vector<Photon> KDTree::nearestPhotons(const Vec3 &position, const int num) const
	{
		vector<Photon> result;
		stack<Node *> path;
		double distance = 0;
		auto pq = std::priority_queue<Photon, vector<Photon>, ComparePhoton>(ComparePhoton(position));

		Node *pSearch = root, *pBack = nullptr;
		while (pSearch)
		{
			path.push(pSearch);
			if (position[pSearch->split] <= pSearch->photon.position[pSearch->split])
				pSearch = pSearch->left;
			else
				pSearch = pSearch->right;
		}

		_pq_push(pq, path.top()->photon, num);
		path.pop();
		distance = glm::distance(pq.top().position, position);
		
		while (!path.empty())
		{
			pBack = path.top();
			path.pop();

			if (pBack->left == nullptr && pBack->right == nullptr)
			{
				if (glm::distance(pBack->photon.position, position) < distance)
				{
					_pq_push(pq, pBack->photon, num);
					distance = glm::distance(pq.top().position, position);
				}
				continue;
			}
			if(fabs(pBack->photon.position[pBack->split] - position[pBack->split]) < distance)
			{
				if (glm::distance(pBack->photon.position, position) < distance)
				{
					_pq_push(pq, pBack->photon, num);
					distance = glm::distance(pq.top().position, position);
				}
				if (position[pBack->split] <= pBack->photon.position[pBack->split])
					pSearch = pBack->right;
				else
					pSearch = pBack->left;
				if(pSearch)
					path.push(pSearch);
			}
		}
		while (!pq.empty())
		{
			result.push_back(pq.top());
			pq.pop();
		}
		return result;
	}

}