/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PCLPointcloudConverter.hpp"

using namespace depth_map_preprocessing;

PCLPointcloudConverter::PCLPointcloudConverter(std::string const& name)
    : PCLPointcloudConverterBase(name)
{
}

PCLPointcloudConverter::PCLPointcloudConverter(std::string const& name, RTT::ExecutionEngine* engine)
    : PCLPointcloudConverterBase(name, engine)
{
}

PCLPointcloudConverter::~PCLPointcloudConverter()
{
}

void PCLPointcloudConverter::newSampleCallback(const base::Time& ts, const base::samples::DepthMap& depth_map_sample)
{
    std::vector<Eigen::Vector3d> pointcloud;
    if(convertToPointCloud(ts, depth_map_sample, pointcloud))
    {
        if(_encode_remissions.value() && depth_map_sample.remissions.size() >= pointcloud.size())
        {
            pcl::PointCloud<pcl::PointXYZI> pcl_pointcloud;
            pcl_pointcloud.header.stamp = ts.microseconds;
            convertToPCLPointCloud(pointcloud, pcl_pointcloud);
            // add remission values
            for(unsigned i = 0, j = 0; i < depth_map_sample.remissions.size(); i++)
            {
                if(depth_map_sample.isIndexValid(i))
                {
                    pcl_pointcloud.points[j].intensity = depth_map_sample.remissions[i];
                    j++;
                }
            }
            writePCLPointCloud(pcl_pointcloud);
        }
        else
        {
            pcl::PointCloud<pcl::PointXYZ> pcl_pointcloud;
            pcl_pointcloud.header.stamp = ts.microseconds;
            convertToPCLPointCloud(pointcloud, pcl_pointcloud);
            writePCLPointCloud(pcl_pointcloud);
        }
    }
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PCLPointcloudConverter.hpp for more detailed
// documentation about them.

bool PCLPointcloudConverter::configureHook()
{
    if (! PCLPointcloudConverterBase::configureHook())
        return false;
    return true;
}
bool PCLPointcloudConverter::startHook()
{
    if (! PCLPointcloudConverterBase::startHook())
        return false;
    return true;
}
void PCLPointcloudConverter::updateHook()
{
    PCLPointcloudConverterBase::updateHook();
}
void PCLPointcloudConverter::errorHook()
{
    PCLPointcloudConverterBase::errorHook();
}
void PCLPointcloudConverter::stopHook()
{
    PCLPointcloudConverterBase::stopHook();
}
void PCLPointcloudConverter::cleanupHook()
{
    PCLPointcloudConverterBase::cleanupHook();
}
