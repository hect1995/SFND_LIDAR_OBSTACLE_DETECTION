#ifndef CLUSTER_H_
#define CLUSTER_H_

#include "../../render/render.h"
#include "../../render/box.h"
#include "kdtree.h"
#include <chrono>
#include <string>
#include <vector>

struct Cluster
{
	// Arguments:
	// window is the region to draw box around
	// increase zoom to see more of the area
	static pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom)
	{
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
		viewer->setBackgroundColor (0, 0, 0);
		viewer->initCameraParameters();
		viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
		viewer->addCoordinateSystem (1.0);

		viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 1, 1, 1, "window");
		return viewer;
	}

	static pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<std::vector<float>> points)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
		
		for(int i = 0; i < points.size(); i++)
		{
			pcl::PointXYZ point;
			point.x = points[i][0];
			point.y = points[i][1];
			point.z = 0;

			cloud->points.push_back(point);

		}
		cloud->width = cloud->points.size();
		cloud->height = 1;

		return cloud;

	}


	static void render2DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth)
	{

		if(node!=NULL)
		{
			Box upperWindow = window;
			Box lowerWindow = window;
			// split on x axis
			if(depth%2==0)
			{
				viewer->addLine(pcl::PointXYZ(node->point[0], window.y_min, 0),pcl::PointXYZ(node->point[0], window.y_max, 0),0,0,1,"line"+std::to_string(iteration));
				lowerWindow.x_max = node->point[0];
				upperWindow.x_min = node->point[0];
			}
			// split on y axis
			else
			{
				viewer->addLine(pcl::PointXYZ(window.x_min, node->point[1], 0),pcl::PointXYZ(window.x_max, node->point[1], 0),1,0,0,"line"+std::to_string(iteration));
				lowerWindow.y_max = node->point[1];
				upperWindow.y_min = node->point[1];
			}
			iteration++;

			render2DTree(node->left,viewer, lowerWindow, iteration, depth+1);
			render2DTree(node->right,viewer, upperWindow, iteration, depth+1);


		}

	}

	static void proximity(const std::vector<std::vector<float>>& points, int id, std::vector<int> &cluster, std::vector<bool> &processed,
		KdTree* tree, const float &distanceTol)
	{
		if (processed[id]){return;}
		processed[id] = true;
		cluster.emplace_back(id);
		std::vector<int> nearby_points = tree->search(points[id], distanceTol);
		for (auto nearby : nearby_points)
		{
			if (!processed[nearby]) // not processed
			{
				proximity(points, nearby, cluster, processed, tree, distanceTol);
			}
		}
	}

	static std::vector<pcl::PointIndices> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize)
	{
		std::vector<pcl::PointIndices> cluster_indices;

		if(points.size() < minSize){
			return cluster_indices;
		}
		// TODO: Fill out this function to return list of indices for each cluster
		std::vector<bool> processed(points.size(),false); 
		for (int i=0; i<points.size(); i++)
		{
			if (processed[i]){continue;} // not processed
			std::vector<int> cluster;
			proximity(points, i, cluster, processed, tree, distanceTol);
			if(cluster.size() < minSize || cluster.size() > maxSize){continue;}
			pcl::PointIndices indices;
			indices.indices = std::move(cluster);
			cluster_indices.push_back(indices);
		}
	
		return cluster_indices;

	}
};

#endif /* CLUSTER_H_ */