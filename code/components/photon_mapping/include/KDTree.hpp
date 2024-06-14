#ifndef __KDTREE_HPP__
#define __KDTREE_HPP__
#include "scene/Scene.hpp"
#include "Ray.hpp"


namespace PhotonMapper
{
	using namespace NRenderer;
	using namespace std;
	struct Photon {
		Vec3 position;
		RGB power;
		Ray in_ray;
		Ray out_ray;
		Vec3 normal;
	};

	class KDTree
	{
	private:
		struct Node
		{
			Node* left = nullptr;
			Node* right = nullptr;
			Photon photon;
			int split = 0;
		};
		Node* root = nullptr;

	public:
		KDTree() = default;
		~KDTree();

		void build(const std::vector<Photon>& photons);
	private:
		void _build(std::vector<Photon>::iterator begin, std::vector<Photon>::iterator end, Node*& node);
		vector<Photon>::iterator _split(std::vector<Photon>::iterator begin, std::vector<Photon>::iterator end, Node*& node);
		void _release(Node* node);
		int chooseSplit(std::vector<Photon>::iterator begin, std::vector<Photon>::iterator end);
	};
}

#endif // __KDTREE_HPP__