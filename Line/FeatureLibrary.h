#pragma once
#include <string>
#include <opencv2/core/mat.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/objdetect.hpp>
#include <map>
#include <opencv2/imgproc.hpp>


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
	enum class DetectorType
	{
		SURFIllumCanny,
		SURF,
		Cascacade
	};

	FeatureLibrary(DetectorType type, int minHessian = 400);
	~FeatureLibrary();

	void Add(FeatureType type, const std::string &name, const std::string &path);
	void Add(FeatureType type, const std::string &name, cv::Mat image);
	void AddCascade(FeatureType feature, const std::string &name, const std::string &path);
	
	void SetTypeOverlayColor(FeatureType type, const cv::Scalar &color);
	const cv::Scalar &GetOverlayColor(FeatureType type) const;

	const std::vector<cv::KeyPoint> &GetKeypoints(FeatureType type, const std::string &name) const;
	const cv::Mat &GetDescriptors(FeatureType type, const std::string &name) const;

	void PreProcess(cv::Mat & image, cv::Mat& mat);
	std::string FindMatch(cv::Mat & image, double *avgDist = nullptr, double *minDist = nullptr, double *maxDist = nullptr);
	std::string FindMatch(FeatureType type, cv::Mat & image, double *avgDist = nullptr, double *minDist = nullptr, double *maxDist = nullptr);
	bool FindMatch(FeatureType type, const std::string name, cv::Mat & image, double *avgDist = nullptr, double *minDist = nullptr, double *maxDist = nullptr);
private:
	void PreProcessIllumCanny(cv::Mat &image, cv::Mat& mat);
	void PreProcessCascade(cv::Mat & image, cv::Mat& mat);
	bool FindMatchSURFIllumCanny(FeatureType type, const std::string name, cv::Mat & image, double *avgDist = nullptr, double *minDist = nullptr, double *maxDist = nullptr);
	bool FindMatchCascade(FeatureType type, const std::string name, cv::Mat & image, double *avgDist = nullptr, double *minDist = nullptr, double *maxDist = nullptr);


	struct featureInfo
	{
		std::vector<cv::KeyPoint> keypoints;
		cv::CascadeClassifier classifier;
		cv::Mat descriptor;
		cv::Mat image;
	};

	struct typeInfo
	{
		cv::Scalar overlayColor;
		std::map<std::string, struct featureInfo> objects;
	};
	DetectorType _type;
	cv::Ptr<cv::xfeatures2d::SURF> _detector;
	cv::Ptr<cv::DescriptorMatcher> _matcher;
	cv::Ptr<cv::CLAHE> _clahe;
	std::map<FeatureType, struct typeInfo> _featuremap;
};
