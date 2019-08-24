//
// Created by philipp on 24.08.19.
//

#ifndef LASER_SCAN_ROTATION_FILTER_H
#define LASER_SCAN_ROTATION_FILTER_H

#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>

namespace laser_filters
{

class LaserScanRotationFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
public:
	int index_;

	bool configure()
	{
		index_ = 0;

		if (!getParam("index", index_))
		{
			ROS_ERROR("The index parameter must be set to use this filter.");
			return false;
		}

		return true;
	}

	virtual ~LaserScanRotationFilter()
	{}

	bool update(const sensor_msgs::LaserScan & input_scan, sensor_msgs::LaserScan & filtered_scan)
	{
		if (input_scan.ranges.size() != input_scan.intensities.size())
		{
			ROS_ERROR("ranges and intensities of input scan must have the same size");
		}

		filtered_scan = input_scan;
		std::size_t size = input_scan.ranges.size();

		for (int i = 0; i < size; ++i)
		{
			filtered_scan.ranges[i] = input_scan.ranges[(index_ + i) % size];
			filtered_scan.intensities[i] = input_scan.intensities[(index_ + i) % size];
		}

		return true;

	}
};

}

#endif //LASER_SCAN_ROTATION_FILTER_H
