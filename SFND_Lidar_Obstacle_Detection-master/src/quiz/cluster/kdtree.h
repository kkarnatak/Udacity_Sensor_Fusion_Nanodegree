/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node** node, int depth, std::vector<float> point, int id)
	{
		if (*node == NULL)
		{
			*node = new Node(point, id);
		}
		else
		{
			uint loc = depth % 2;

			// Fetch the location to insert:
			// If Even pick X, Odd pick Y

			if ( point[loc] < (*node)->point[loc] )
			{
				insertHelper( &(*node)->left, depth + 1, point, id);
			}
			else
			{
				insertHelper(&(*node)->right, depth + 1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

		insertHelper(&root, 0, point, id);
	}

	void searchHelper(Node* node, uint depth, std::vector<float> target, float distanceTol, std::vector<int>& ids)
	{
		if (node != NULL)
		{
			if ( (node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol))
				&& (node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol)) )
			{
				float distance = std::sqrt(pow(node->point[0] - target[0], 2) + pow(node->point[1] - target[1], 2));

				if (distance <= distanceTol)
					ids.push_back(node->id);
			}

			if ((target[depth % 2] - distanceTol) < node->point[depth % 2])
				searchHelper(node->left, depth + 1, target, distanceTol, ids);
			if ((target[depth % 2] + distanceTol) > node->point[depth % 2])
				searchHelper(node->right, depth + 1, target, distanceTol, ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, 0, target, distanceTol, ids);
		return ids;
	}
	

};




