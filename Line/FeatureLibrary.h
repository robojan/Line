#pragma once
#include <string>
#include <opencv2/core/mat.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <map>


class ImageReadException : public std::runtime_error {
public:
	explicit ImageReadException(char const* const message) throw() : std::runtime_error(message) {}
	ImageReadException(ImageReadException const& other) throw() : std::runtime_error(other) {}
};

enum class FeatureType
{
	BlueSign,
	RedSign,
	YellowSign
};

class FeatureLibrary
{
public:
	FeatureLibrary(int minHessian = 400);
	~FeatureLibrary();

	void Add(FeatureType type, const std::string &name, const std::string &path);
	void Add(FeatureType type, const std::string &name, cv::Mat image);
	
	void SetTypeOverlayColor(FeatureType type, const cv::Scalar &color);
	const cv::Scalar &GetOverlayColor(FeatureType type) const;

	const std::vector<cv::KeyPoint> &GetKeypoints(FeatureType type, const std::string &name) const;
	const cv::Mat &GetDescriptors(FeatureType type, const std::string &name) const;

	std::string FindMatch(cv::InputArray image, double *avgDist = nullptr, double *minDist = nullptr, double *maxDist = nullptr);
	std::string FindMatch(FeatureType type, cv::InputArray image, double *avgDist = nullptr, double *minDist = nullptr, double *maxDist = nullptr);
	bool FindMatch(FeatureType type, const std::string name, cv::InputArray image, double *avgDist = nullptr, double *minDist = nullptr, double *maxDist = nullptr);
private:
	struct featureInfo
	{
		std::vector<cv::KeyPoint> keypoints;
		cv::Mat descriptor;
		cv::Mat image;
	};

	struct typeInfo
	{
		cv::Scalar overlayColor;
		std::map<std::string, struct featureInfo> objects;
	};

	cv::Ptr<cv::xfeatures2d::SURF> _detector;
	cv::Ptr<cv::DescriptorMatcher> _matcher;
	std::map<FeatureType, struct typeInfo> _featuremap;
};
