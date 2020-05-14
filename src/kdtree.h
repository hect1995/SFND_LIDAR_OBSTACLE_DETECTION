/* \author Aaron Brown */
// Quiz on implementing kd tree

#ifndef KDTREE_H_
#define KDTREE_H_
#include "render/render.h"


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

	void insertHelper(Node **node, int level, std::vector<float> point, int id)
   {
	  int index_to_care = level%2;

      if(*node == NULL)
      {
		Node* nd = new Node(point,id);
		*node = nd;
      }
      else if(point[index_to_care] < (*node)->point[index_to_care])
      {
        insertHelper(&(*node)->left, ++level, point, id);
      }
      else
      {
        insertHelper(&(*node)->right, ++level, point, id);
      }
   }

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);
	}


	void searchHelper(std::vector<float> target, float distanceTol, Node **node, int level, std::vector<int> &ids)
	{
		if (*node==NULL){return;}
		if (((*node)->point[0] < target[0] + distanceTol && (*node)->point[0] > target[0] - distanceTol) &&
			((*node)->point[1] < target[1] + distanceTol && (*node)->point[1] > target[1] - distanceTol)) // inside the box
		{
			ids.push_back((*node)->id);
		}
		int index_to_care = level%2;
		if ((*node)->point[index_to_care] < target[index_to_care] + distanceTol)
		{
			searchHelper(target, distanceTol, &(*node)->right, level+1, ids);
		}
		if ((*node)->point[index_to_care] > target[index_to_care] - distanceTol)
		{
			searchHelper(target, distanceTol, &(*node)->left, level+1, ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, distanceTol, &root, 0, ids);
		return ids;
	}
	

};

#endif /* KDTREE_H_ */


