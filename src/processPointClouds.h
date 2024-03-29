// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <unordered_set>



/*
 * Struct: Node
 * ------------
 * It used to represent node of kd tree
 */

struct Node
{
    pcl::PointXYZI point;
    int id;
    Node* left;
    Node* right;

    Node(pcl::PointXYZI ptr, int setId):point(ptr), id(setId), left(NULL), right(NULL){}
};


/*
 * Struct: Kdtree
 * --------------
 * This interface exports the KdTree struct, which separating space by splitting regions,
 * nearest neighbor search can be made much faster when using an algorithm like euclidean clustering. 
 */
 
struct KdTree
{
    Node* root;
    KdTree():root(NULL){}

    void insertHelper(Node** node, int depth, pcl::PointXYZI point, int id)
    {
        //tree is empty
		if(*node==NULL)
			*node = new Node(point, id);
		else
		{
			uint cd = depth % 2;

			if(cd==0){
                if(point.x < (*node)->point.x) 
                    insertHelper(&(*node)->left, depth + 1, point, id);
                else 
                    insertHelper(&(*node)->right, depth + 1, point, id);
            }else{
                if(point.y < (*node)->point.y) 
                    insertHelper(&(*node)->left, depth + 1, point, id);
                else 
                    insertHelper(&(*node)->right, depth + 1, point, id);
            }
		}
    }

    void insert(pcl::PointXYZI point, int id)
	{
		insertHelper(&root, 0, point, id);
	}

    void searchHelper(pcl::PointXYZI pivot, Node* node, int depth, float distanceTol, std::vector<int>& ids)
    {
        if(node != NULL)
        {
            if((node->point.x >= (pivot.x-distanceTol)&&(node->point.x <= (pivot.x+distanceTol)))&&(node->point.y >= (pivot.y-distanceTol)&&(node->point.y <= (pivot.y+distanceTol))))
            {
                float distance = sqrt((node->point.x - pivot.x) * (node->point.x - pivot.x) + (node->point.y - pivot.y) * (node->point.y - pivot.y));
                if(distance <= distanceTol) ids.push_back(node->id);
            }
            if(depth%2 == 0){
                if((pivot.x - distanceTol) < node->point.x) searchHelper(pivot, node->left, depth+1, distanceTol, ids);
                if((pivot.x + distanceTol) > node->point.x) searchHelper(pivot, node->right, depth+1, distanceTol, ids);
            }
            else{
                if((pivot.y - distanceTol) < node->point.y) searchHelper(pivot, node->left, depth+1, distanceTol, ids);
                if((pivot.y + distanceTol) > node->point.y) searchHelper(pivot, node->right, depth+1, distanceTol, ids);
            }
        
        }
    }

    // return a list of point ids in the tree that are within distance of pivot
	std::vector<int> search(pcl::PointXYZI pivot, float distanceTol)
	{
		std::vector<int> ids;
        searchHelper(pivot, root, 0, distanceTol, ids);
		return ids;
	}

};

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    //Add the function euclideanCluster

    void clusterHelper(int idx, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, int minSize, int maxSize);

    
    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */