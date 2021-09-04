#ifndef DATATYPE
#define DATATYPE

#include <QVector>
#include <QMetaType>

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/ppf.h>

# define PI  3.1415926

typedef QVector<float> MyArray;

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud<PointType> PointCloudType;
typedef pcl::Normal NormalType;
typedef pcl::PointCloud<NormalType> PointCloudNormalType;
typedef pcl::ReferenceFrame RFType;

typedef pcl::SHOT352 DescriptorTypeSHOT;
typedef pcl::SHOTEstimationOMP<pcl::PointXYZ, NormalType, DescriptorTypeSHOT> EstimatorTypeSHOT;

typedef pcl::PPFSignature DescriptorTypePPF;
typedef pcl::PPFEstimation<pcl::PointXYZ, NormalType, DescriptorTypePPF> EstimatorTypePPF;

typedef pcl::PointNormal PointXYZTangent;

struct CloudStyle
{
	double r;
	double g;
	double b;
	double size;

	CloudStyle(double r,
		double g,
		double b,
		double size) :
		r(r),
		g(g),
		b(b),
		size(size)
	{
	}
};

extern CloudStyle style_white;
extern CloudStyle style_red;
extern CloudStyle style_green;
extern CloudStyle style_cyan;
extern CloudStyle style_violet;

class RobotPosition
{
private:
	double x;
	double y;
	double z;
	double r;
	double p;
	double w;
	double size;

public:
	RobotPosition(double x,
		double y,
		double z,
		double r,
		double p,
		double w) :
		x(x),
		y(y),
		z(z),
		r(r),
		p(p),
		w(w)
	{
	}

	RobotPosition() :
		x(0),
		y(0),
		z(0),
		r(0),
		p(0),
		w(0)
	{
	}

	using Ptr = std::shared_ptr<RobotPosition>;

	inline Ptr
      makeShared () const { return Ptr (new RobotPosition (*this)); }
};



#endif