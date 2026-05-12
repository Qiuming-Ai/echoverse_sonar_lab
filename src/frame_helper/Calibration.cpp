#include <fstream>
#include "Calibration.h"
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>
#include <sstream>
#include <iterator>
#include <iostream>

using namespace frame_helper;
using namespace std;

CameraCalibration::CameraCalibration()
    : fx(sonar_types_v2::unset<double>()), fy(sonar_types_v2::unset<double>()), cx(sonar_types_v2::unset<double>()), cy(sonar_types_v2::unset<double>()), 
    d0(sonar_types_v2::unset<double>()), d1(sonar_types_v2::unset<double>()), d2(sonar_types_v2::unset<double>()), d3(sonar_types_v2::unset<double>()),
    width(-1), height(-1), ex(sonar_types_v2::unset<double>()), ey(sonar_types_v2::unset<double>()),fisheye(false)
{}

CameraCalibration::CameraCalibration( 
        double fx, double fy, double cx, double cy, 
        double d0, double d1, double d2, double d3, 
        int width, int height )
    : fx(fx), fy(fy), cx(cx), cy(cy), 
    d0(d0), d1(d1), d2(d2), d3(d3),
    width(width), height(height),  
    ex(sonar_types_v2::unset<double>()), ey(sonar_types_v2::unset<double>()),fisheye(false)
{}

Eigen::Matrix3d CameraCalibration::getCameraMatrix() const
{
    Eigen::Matrix3d res;
    res << fx, 0, cx,
        0, fy, cy,
        0, 0, 1.0;
    return res;
}

Eigen::Matrix2d CameraCalibration::getPixelCovariance() const
{
    Eigen::Matrix2d res;
    res << ex, 0.0,
        0.0, ey;
    return res;
}

void CameraCalibration::rescale( int width, int height )
{
    double sx = (double)width / (double)this->width;
    double sy = (double)height / (double)this->height;

    fx *= sx;
    cx *= sx;
    fy *= sy;
    cy *= sy;
}

bool CameraCalibration::isValid() const
{
    return 
        width > 0 &&
        height > 0 &&
        !sonar_types_v2::isUnset<double>(fx) &&	
        !sonar_types_v2::isUnset<double>(fy) &&	
        !sonar_types_v2::isUnset<double>(cx) &&	
        !sonar_types_v2::isUnset<double>(cy) &&	
        !sonar_types_v2::isUnset<double>(d0) &&	
        !sonar_types_v2::isUnset<double>(d1) &&	
        !sonar_types_v2::isUnset<double>(d2) &&	
        !sonar_types_v2::isUnset<double>(d3);
}

CameraCalibration CameraCalibration::fromFrame( const sonar_types_v2::samples::frame::Frame& frame )
{
    CameraCalibration c;
    c.fx = frame.getAttribute<double>("fx");
    c.fy = frame.getAttribute<double>("fy");
    c.cx = frame.getAttribute<double>("cx");
    c.cy = frame.getAttribute<double>("cy");
    c.d0 = frame.getAttribute<double>("d0");
    c.d1 = frame.getAttribute<double>("d1");
    c.d2 = frame.getAttribute<double>("d2");
    c.d3 = frame.getAttribute<double>("d3");
    c.width = frame.size.width;
    c.height = frame.size.height;
    return c;
}

void CameraCalibration::toFrame( sonar_types_v2::samples::frame::Frame& frame ) const
{
    frame.setAttribute<double>("fx", fx);
    frame.setAttribute<double>("fy", fy);
    frame.setAttribute<double>("cx", cx);
    frame.setAttribute<double>("cy", cy);
    frame.setAttribute<double>("d0", d0);
    frame.setAttribute<double>("d1", d1);
    frame.setAttribute<double>("d2", d2);
    frame.setAttribute<double>("d3", d3);

    if( frame.size.width != width )
        throw std::runtime_error("frame width does not match calibration");
    if( frame.size.height != height )
        throw std::runtime_error("frame height does not match calibration");
}

ExtrinsicCalibration::ExtrinsicCalibration()
: tx(sonar_types_v2::unset<double>()), ty(sonar_types_v2::unset<double>()), tz(sonar_types_v2::unset<double>()), 
    rx(sonar_types_v2::unset<double>()), ry(sonar_types_v2::unset<double>()), rz(sonar_types_v2::unset<double>())
{}

ExtrinsicCalibration::ExtrinsicCalibration( double tx, double ty, double tz, double rx, double ry, double rz )
: tx(tx), ty(ty), tz(tz), 
    rx(rx), ry(ry), rz(rz)
{}

bool ExtrinsicCalibration::isValid() const
{
    return 
        !sonar_types_v2::isUnset<double>(tx) &&	
        !sonar_types_v2::isUnset<double>(ty) &&	
        !sonar_types_v2::isUnset<double>(tz) &&	
        !sonar_types_v2::isUnset<double>(rx) &&	
        !sonar_types_v2::isUnset<double>(ry) &&	
        !sonar_types_v2::isUnset<double>(rz);
}

Eigen::Affine3d ExtrinsicCalibration::getTransform() const
{
    Eigen::Vector3d t( tx, ty, tz ), r( rx, ry, rz );
    Eigen::Affine3d result = Eigen::Affine3d::Identity();
    if( r.norm() > 0 )
	result.linear() = Eigen::AngleAxisd( r.norm(), r.normalized() ).toRotationMatrix();

    result.translation() = t;

    return result;
}

StereoCalibration StereoCalibration::fromMatlabFile( const std::string& file_name )
{
    return StereoCalibration::fromMatlabFile( file_name, -1, -1 );
}

StereoCalibration::StereoCalibration()
{}

StereoCalibration::StereoCalibration( const CameraCalibration& left, const CameraCalibration& right, const ExtrinsicCalibration& extrinsic )
    : camLeft( left ), camRight( right ), extrinsic( extrinsic )
{}

StereoCalibration StereoCalibration::fromMatlabFile( const std::string& file_name, int width, int height )
{
    ifstream ifs( file_name.c_str() );
    string line;

    // strategy is to read the input matlab file into 
    // a map, which contains the key (matlab variable)
    // and stores the vector as a vector of floats
    map<string, vector<double> > raw;

    boost::regex e( "(\\w+?) = \\[([^\\]]+?)\\]" );
    boost::smatch what;

    while( getline( ifs, line ) )
    {
	if( boost::regex_search( line, what, e ) )
	{
	    if( what.size() == 3 )
	    {
		raw[what[1]] = vector<double>();
		istringstream iss( what[2] );
		istream_iterator<string> is( iss );
		for(; is != istream_iterator<string>(); is++ )
		    raw[what[1]].push_back( boost::lexical_cast<double>( *is ) );
	    }
	}
    }

    // check if all fields are there
    assert( raw["fc_left"].size() >= 2 );
    assert( raw["cc_left"].size() >= 2 );
    assert( raw["kc_left"].size() >= 4 );

    assert( raw["fc_right"].size() >= 2 );
    assert( raw["cc_right"].size() >= 2 );
    assert( raw["kc_right"].size() >= 4 );

    assert( raw["T"].size() >= 3 );
    assert( raw["om"].size() >= 3 );

    // now we can fill the calibration based on the matlab
    // vectors we have obtained
    StereoCalibration result = 
    StereoCalibration(
	CameraCalibration( raw["fc_left"][0], raw["fc_left"][1], 
	  raw["cc_left"][0], raw["cc_left"][1], 
	  raw["kc_left"][0], raw["kc_left"][1], raw["kc_left"][2], raw["kc_left"][3],
          width, height ),
	CameraCalibration( raw["fc_right"][0], raw["fc_right"][1], 
	  raw["cc_right"][0], raw["cc_right"][1], 
	  raw["kc_right"][0], raw["kc_right"][1], raw["kc_right"][2], raw["kc_right"][3],
          width, height ),
        ExtrinsicCalibration(
	  raw["T"][0], raw["T"][1], raw["T"][2],
	  raw["om"][0], raw["om"][1], raw["om"][2] )
        );

    return result;
}

bool StereoCalibration::isValid() const
{
    return
        camLeft.isValid() &&
        camRight.isValid() &&
        extrinsic.isValid();
}

