#include <apc_grasping/pcl_filters.h>

namespace pcl_filters {

PointCloud::Ptr RadiusOutlierRemoval(PointCloud::Ptr cloud, std::string method,
                                     int kNeighbours, float radius) {
    PointCloud::Ptr cloud_filtered(new PointCloud);

    if (method == "Radius") {
        pcl::RadiusOutlierRemoval<PointT> outrem;
        // build the filter
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(radius);
        outrem.setMinNeighborsInRadius(kNeighbours);
        // apply filter
        outrem.filter(*cloud_filtered);
    }
    // else if (method == "Conditional")
    // {
    //     // build the condition
    //     pcl::ConditionAnd<PointT>::Ptr range_cond (new
    //     pcl::ConditionAnd<PointT> ());
    //
    //     range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr
    //     (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GT,
    //     0.0)));
    //     range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr
    //     (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::LT,
    //     radius)));
    //
    //     // build the filter
    //     pcl::ConditionalRemoval<PointT> condrem (range_cond);
    //     condrem.setInputCloud (cloud);
    //     condrem.setKeepOrganized(true);
    //     // apply filter
    //     condrem.filter (*cloud_filtered);
    // }

    return cloud_filtered;

    //     std::cerr << "Cloud before filtering: " << std::endl;
    //     for (size_t i = 0; i < cloud->points.size (); ++i)
    //       std::cerr << "    " << cloud->points[i].x << " "
    //                           << cloud->points[i].y << " "
    //                           << cloud->points[i].z << std::endl;
    //     // display pointcloud after filtering
    //     std::cerr << "Cloud after filtering: " << std::endl;
    //     for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
    //       std::cerr << "    " << cloud_filtered->points[i].x << " "
    //                           << cloud_filtered->points[i].y << " "
    //                           << cloud_filtered->points[i].z << std::endl;
}

pcl::PointCloud<pcl::Boundary>::Ptr boundaryEstimation(
    pcl::PointCloud<pcl::PointNormal>::Ptr input, int k, double radius,
    double angle) {
    // Convert data to PointCloud<T>
    //  pcl::PointCloud<pcl::PointNormal>::Ptr xyznormals (new
    //  pcl::PointCloud<pcl::PointNormal>);
    // fromPCLPointCloud2 (*input, *xyznormals);

    // Estimate
    pcl::console::TicToc tt;
    tt.tic();

    pcl::console::print_highlight(stderr, "Computing ");

    pcl::BoundaryEstimation<pcl::PointNormal, pcl::PointNormal, pcl::Boundary>
        ne;
    ne.setInputCloud(input);
    ne.setInputNormals(input);
    // ne.setSearchMethod (pcl::KdTreeFLANN<pcl::PointNormal>::Ptr (new
    // pcl::KdTreeFLANN<pcl::PointNormal>));
    ne.setKSearch(k);
    ne.setAngleThreshold(static_cast<float>(angle));
    ne.setRadiusSearch(radius);

    pcl::PointCloud<pcl::Boundary>::Ptr boundaries(
        new pcl::PointCloud<pcl::Boundary>);
    ne.compute(*boundaries);

    pcl::console::print_info("[done, ");
    pcl::console::print_value("%g", tt.toc());
    pcl::console::print_info(" ms : ");
    pcl::console::print_value("%d", boundaries->width * boundaries->height);
    pcl::console::print_info(" points]\n");

    // Convert data back
    // pcl::PCLPointCloud2 output_boundaries, output;
    // toPCLPointCloud2 (boundaries, output_boundaries);

    //  pcl::PCLPointCloud2 inputPCL2, boundariesPCL2;
    //  pcl::PCLPointCloud2::Ptr output(new pcl::PCLPointCloud2);

    //  pcl::toPCLPointCloud2 (input, inputPCL2);
    //  pcl::toPCLPointCloud2 (boundaries, boundariesPCL2);

    //  pcl::concatenateFields (inputPCL2, boundariesPCL2, output);

    return boundaries;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr StatisticalOutlierRemoval(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int meanK, float std) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZ>);

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(meanK);  // 50
    sor.setStddevMulThresh(std);
    sor.filter(*cloud_filtered);

    // std::cerr << "Cloud after filtering: " << std::endl;
    // std::cerr << *cloud_filtered << std::endl;

    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr smooth_cloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius) {
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointNormal>::Ptr output_cloud(
        new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_xyz_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    // mls.setComputeNormals (true);

    // Set parameters
    mls.setInputCloud(cloud);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(radius);
    mls.process(*output_cloud);

    pcl::copyPointCloud(*output_cloud, *output_xyz_cloud);

    return output_xyz_cloud;
}

 void compute_normals_down_sampled(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr downSampledCloud, pcl::PointCloud<pcl::Normal>::Ptr normals, Eigen::Vector3f view_point, float radius) {
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(downSampledCloud);

    // Pass the original data (before downsampling) as the search surface
    ne.setSearchSurface(cloud);

    // Create an empty kdtree representation, and pass it to the normal
    // estimation object.
    // Its content will be filled inside the object, based on the given surface
    // dataset.
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    // Output datasets
    // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
    //     new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(radius);

    //  if(flip_orientation) ne.setViewPoint (1.0, 1.0, 1.0);
    // ne.setViewPoint(0.0, 0.0, 1.0);
    ne.setViewPoint(view_point[0], view_point[1], view_point[2]);

    // Compute the features
    ne.compute(*normals);

    // return cloud_normals;
}

pcl::PointCloud<pcl::Normal>::Ptr compute_normals(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius) {
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    // Create an empty kdtree representation, and pass it to the normal
    // estimation object.
    // Its content will be filled inside the object, based on the given input
    // dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
        new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(radius);

    // Compute the features
    ne.compute(*cloud_normals);

    // ne.setViewPoint (float vpx, float vpy, float vpz);

    return cloud_normals;

    // cloud_normals->points.size () should have the same size as the input
    // cloud->points.size ()*
}

// PointCloudWithNormals::Ptr compute_normals(PointCloud::Ptr cloud, float
// radius){

//    // Create the normal estimation class, and pass the input dataset to it
//    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
//    ne.setInputCloud (cloud);

//    // Create an empty kdtree representation, and pass it to the normal
//    estimation object.
//    // Its content will be filled inside the object, based on the given input
//    dataset (as no other search surface is given).
//    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new
//    pcl::search::KdTree<pcl::PointXYZRGB> ());
//    ne.setSearchMethod (tree);

//    // Output datasets
//    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new
//    pcl::PointCloud<pcl::Normal>);

//    // Use all neighbors in a sphere of radius 3cm
//    ne.setRadiusSearch (0.03);

//    // Compute the features
//    ne.compute (*cloud_normals);

//    return cloud_normals;

//    // cloud_normals->points.size () should have the same size as the input
//    cloud->points.size ()*
//}

// returns a vector of clusters extracted from input cloud based on euclidean
// tolerance and min/max cluster size
std::vector<PointCloud::Ptr> euclideanClusterRemoval(PointCloud::Ptr cloud_rgb,
                                                     float ClusterTolerance,
                                                     int minClusterSize,
                                                     int MaxClusterSize) {
    std::vector<PointCloud::Ptr> clusters;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_rgb, *cloud_xyz);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_xyz);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(ClusterTolerance);  // 2cm
    ec.setMinClusterSize(minClusterSize);      // 100
    ec.setMaxClusterSize(MaxClusterSize);      // 25000
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_xyz);  // cloud_filtered
    ec.extract(cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it =
             cluster_indices.begin();
         it != cluster_indices.end(); ++it) {
        PointCloud::Ptr cloud_cluster(new PointCloud);
        for (std::vector<int>::const_iterator pit = it->indices.begin();
             pit != it->indices.end(); ++pit)
            cloud_cluster->points.push_back(cloud_rgb->points[*pit]);  //*

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }

    return clusters;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_based_region_growing(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree =
        boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> >(
            new pcl::search::KdTree<pcl::PointXYZRGB>);

    // pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud
    // <pcl::PointXYZRGB>);

    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0);
    pass.filter(*indices);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud(cloud);
    reg.setIndices(indices);
    reg.setSearchMethod(tree);
    reg.setDistanceThreshold(10);
    reg.setPointColorThreshold(6);
    reg.setRegionColorThreshold(5);
    reg.setMinClusterSize(600);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    return reg.getColoredCloud();
}

PointCloud::Ptr regionGrowingSegmentation(PointCloud::Ptr cloud) {
    pcl::search::Search<PointT>::Ptr tree =
        boost::shared_ptr<pcl::search::Search<PointT> >(
            new pcl::search::KdTree<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0);
    pass.filter(*indices);

    pcl::RegionGrowing<PointT, pcl::Normal> reg;
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setInputCloud(cloud);
    // reg.setIndices (indices);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    std::cout << "Number of clusters is equal to " << clusters.size()
              << std::endl;
    std::cout << "First cluster has " << clusters[0].indices.size()
              << " points." << std::endl;
    std::cout << "These are the indices of the points of the initial"
              << std::endl
              << "cloud that belong to the first cluster:" << std::endl;
    unsigned int counter = 0;
    while (counter < clusters[0].indices.size()) {
        std::cout << clusters[0].indices[counter] << ", ";
        counter++;
        if (counter % 10 == 0) std::cout << std::endl;
    }
    std::cout << std::endl;

    return reg.getColoredCloud();
}

PointCloud::Ptr minCutBasedSegmentation(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr object, float sigma, float radius,
    int kNeighbours, float weight) {
    // pcl::PointCloud <pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud
    // <pcl::PointXYZ>);

    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0);
    pass.filter(*indices);

    pcl::MinCutSegmentation<pcl::PointXYZ> seg;
    seg.setInputCloud(cloud);
    seg.setIndices(indices);

    //      pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new
    //      pcl::PointCloud<pcl::PointXYZ> ());
    //      pcl::PointXYZ point;
    //      point.x = 68.97;
    //      point.y = -18.55;
    //      point.z = 0.57;
    //      foreground_points->points.push_back(point);
    seg.setForegroundPoints(object);

    seg.setSigma(0.25);
    seg.setRadius(3.0433856);
    seg.setNumberOfNeighbours(14);
    seg.setSourceWeight(0.8);

    std::vector<pcl::PointIndices> clusters;
    seg.extract(clusters);

    std::cout << "Maximum flow is " << seg.getMaxFlow() << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud =
        seg.getColoredCloud();
    //   pcl::visualization::CloudViewer viewer ("Cluster viewer");
    //   viewer.showCloud(colored_cloud);
    //   while (!viewer.wasStopped ())
    //   {
    //   }
    return seg.getColoredCloud();
}

// Create the filtering object: downsample the dataset using a leaf size of 1cm
}

namespace ros_pcl_tools {

void readCloud(const sensor_msgs::PointCloud2::ConstPtr msg,
               PointCloud::Ptr cloud) {
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*msg, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, *cloud);
}

void writeCloudtoMsg(PointCloud::Ptr cloud, sensor_msgs::PointCloud2::Ptr msg) {
    pcl::PCLPointCloud2 pcl_pc;
    pcl::toPCLPointCloud2(*cloud, pcl_pc);
    pcl_conversions::fromPCL(pcl_pc, *msg);
}

void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) {
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
    pCvImage->image.copyTo(image);
}
}
