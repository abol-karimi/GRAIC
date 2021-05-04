// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <set>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>

// TODO: replace with boost types (geometry, polygon, or uBLAS::vector)
struct PointFloat {
	float x;
	float y;
	PointFloat(float x0, float y0) : x(x0), y(y0) {}
};

struct SegmentFloat {
	PointFloat p0;
	PointFloat p1;
	SegmentFloat(float x1, float y1, float x2, float y2) : p0(x1, y1), p1(x2, y2) {}
};

/**
 * 
 */
class Perception
{
public:
	Perception();
	
	const std::vector<segment_type>& GetWalls(const sensor_msgs::LaserScan::ConstPtr& msg);

private:
	void Polylinize(std::vector<segment_type>& OutLineSegments, float DiscontinuityThreshold); // Convert raw distances to line segments
	bool GetSegment(SegmentFloat& OutSegment, float& OutStartAngle, float StepAngle, float DiscontinuityThreshold);
	bool GetPointAtAngle(PointFloat& OutPoint, float angle_deg); // Calculates the lidar point at angle_deg
	bool GetDistanceAtAngle(float& OutDistance, float angle_deg); // Returns the corresponding distance
	float Distance(PointFloat p0, PointFloat p1);
	float DistanceToLine(PointFloat point, PointFloat p0, PointFloat p1);

	// Controller properties
	float MinTrackWidth = 1.5; // in meters
	float PurepursuitLookahead = 1; // Distance (in meters) between the rear axel and the goal point
	float allowed_obs_dist = 0.3f; // in meters
	float discontinuity_threshold = 0.5f;

	// VoronoiGraph properties
	std::vector<segment_type> Walls;

	// Lidar
	float AngularResolution = 0.25; // 4 measurements per angle
	float Range = 17; // Maximum detectable distance in meters
	float OutOfRange = 65.533; // Value to return if distance > LidarRange
	float LidarMinDegree = -135;
	float LidarMaxDegree = 135;
	std::vector<float> Distances;
};