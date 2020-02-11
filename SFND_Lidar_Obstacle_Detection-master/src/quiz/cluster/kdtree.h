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
			unsigned int loc = depth % 2;

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

	void searchHelper(Node* node, unsigned int depth, std::vector<float> target, float distanceTol, std::vector<int>& ids)
	{
		if (node != NULL)
		{
			float node_X = node->point[0];
			float node_Y = node->point[1];
			float target_X = target[0];
			float target_Y = target[1];
			if ( (node_X >= (target_X - distanceTol) && node_X <= (target_X + distanceTol))
				&& (node_Y >= (target_Y - distanceTol) && node_Y <= (target_Y + distanceTol)) )
			{
				float distance = std::sqrt( std::pow(node_X - target_X, 2) + std::pow(node_Y - target_Y, 2) );

				if (distance < distanceTol)
					ids.push_back(node->id);
			}

			int d_index = depth % 2;
			if ((target[d_index] - distanceTol) < node->point[d_index])
				searchHelper(node->left, depth + 1, target, distanceTol, ids);
			if ((target[d_index] + distanceTol) > node->point[d_index])
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




