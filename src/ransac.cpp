#include <unordered_set>

#include <pcl/common/common.h>

template<typename PointT>
class Ransac {
private:
  int maxIterations;
  float distanceTol;
  int num_points;

public:
  Ransac(int maxIter, float distTol, int nPts) : maxIterations(maxIter), distanceTol(distTol), num_points(nPts) {}
  ~Ransac();
  std::unordered_set<int> Ransac3d(typename pcl::PointCloud<PointT>::Ptr cloud);
};

/**********************************************************************/
template<typename PointT>
Ransac<PointT>::~Ransac() {}

/**********************************************************************/
template<typename PointT>
std::unordered_set<int> Ransac<PointT>::Ransac3d(typename pcl::PointCloud<PointT>::Ptr cloud)
{
  std::unordered_set<int> inlier_found;

  auto cloud_points = cloud->points;

	// walk through all points
	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	while (maxIterations--) {
		std::unordered_set<int> inliers;
		while (inliers.size()<3) {
			inliers.insert(rand()%num_points);
		}

		// use 3-points to define plane
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;
		auto itr = inliers.begin();
		x1 = cloud_points[*itr].x; //cloud->points[*itr].x;
		y1 = cloud_points[*itr].y; //cloud->points[*itr].y;
		z1 = cloud_points[*itr].z; //cloud->points[*itr].z;
		itr++;
		x2 = cloud_points[*itr].x;
		y2 = cloud_points[*itr].y;
		z2 = cloud_points[*itr].z;
		itr++;
		x3 = cloud_points[*itr].x;
		y3 = cloud_points[*itr].y;
		z3 = cloud_points[*itr].z;

		// plane function
		// ax + by + cz + d = 0
		float a, b, c, d, sqrt_abc;
		a = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
		b = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
		c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
		d = - (a*x1 + b*y1 + c*z1);
		sqrt_abc = sqrt(a*a + b*b + c*c);

		
		for (int i=0; i<num_points; i++) {
			if (inliers.count(i)>0) {
				continue;
			}
	  	// first while loop		
      	PointT pt = cloud_points[i];
			float dist = fabs(a*pt.x + b*pt.y + c*pt.z + d)/sqrt_abc;

			if (dist<=distanceTol) {
				inliers.insert(i);
			}
			// During the first while loop
			//try to find a plane that have biggest inlier size
			if (inliers.size()>inlier_found.size()) {
				inlier_found = inliers;
			}
		}
	}

  return inlier_found;
}
