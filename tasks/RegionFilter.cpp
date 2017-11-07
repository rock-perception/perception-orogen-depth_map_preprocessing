/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RegionFilter.hpp"
#include <depth_map_preprocessing/Filters.hpp>

using namespace depth_map_preprocessing;

RegionFilter::RegionFilter(std::string const& name)
    : RegionFilterBase(name)
{
}

RegionFilter::RegionFilter(std::string const& name, RTT::ExecutionEngine* engine)
    : RegionFilterBase(name, engine)
{
}

RegionFilter::~RegionFilter()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See RegionFilter.hpp for more detailed
// documentation about them.

bool RegionFilter::configureHook()
{
    if (! RegionFilterBase::configureHook())
        return false;
        
    segment = base::AngleSegment(base::Angle::fromRad(_mean_angle.value().rad - 0.5 * _angular_width.value()), _angular_width.value());
        
    return true;
}
bool RegionFilter::startHook()
{
    if (! RegionFilterBase::startHook())
        return false;
    return true;
}
void RegionFilter::updateHook()
{
    RegionFilterBase::updateHook();
    
    base::samples::DepthMap depth_map;
    while(_depth_map.read(depth_map, false) == RTT::NewData)
    {
        Filters::filterRegion(depth_map, segment, _distance.value());
        _filtered_depth_map.write(depth_map);
    }
}
void RegionFilter::errorHook()
{
    RegionFilterBase::errorHook();
}
void RegionFilter::stopHook()
{
    RegionFilterBase::stopHook();
}
void RegionFilter::cleanupHook()
{
    RegionFilterBase::cleanupHook();
}
