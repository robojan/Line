#include "FeatureLibrary.h"
#include <opencv2/imgcodecs.hpp>
#include <exception>
#include <opencv2/imgproc.hpp>

FeatureLibrary::FeatureLibrary(DetectorType type, int minHessian /* = 400 */) :
	_type(type)
{
	switch(type)
	{
	case DetectorType::SURFIllumCanny:
		_clahe = cv::createCLAHE();
		_clahe->setClipLimit(2);
	case DetectorType::SURF:
		_matcher = cv::BFMatcher::create(cv::NORM_L2, true);
		_detector = cv::xfeatures2d::SURF::create(minHessian);
	case DetectorType::Cascacade:

		break;
	}
}

FeatureLibrary::~FeatureLibrary()
{
	
}

void FeatureLibrary::Add(FeatureType type, const std::string &name, const std::string &path)
{
	cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
	if(image.empty())
	{
		throw ImageReadException(("Could not open the image file. " + path).c_str());
	}
	cv::cvtColor(image, image, CV_BGR2Lab);
	std::vector<cv::Mat> planes(3);
	cv::split(image, planes);
	Add(type, name, planes[0]);
}

void FeatureLibrary::Add(FeatureType type, const std::string &name, cv::Mat image)
{
	struct featureInfo info;

	cv::Mat illumCorrected;
	switch(_type)
	{
	case DetectorType::SURFIllumCanny:
		_clahe->apply(image, illumCorrected);
		cv::Canny(illumCorrected, illumCorrected, 150, 200, 3);
		_detector->detectAndCompute(illumCorrected, cv::Mat(), info.keypoints, info.descriptor);
		info.image = illumCorrected;
		break;
	case DetectorType::SURF:
		_detector->detectAndCompute(image, cv::Mat(), info.keypoints, info.descriptor);
		info.image = image;
		break;
	default:
		throw std::runtime_error("Error cannot add image");
	}

	_featuremap[type].objects[name] = info;
}

void FeatureLibrary::AddCascade(FeatureType type, const std::string& name, const std::string& path)
{
	struct featureInfo info;
	if(_type != DetectorType::Cascacade)
	{
		throw std::runtime_error("Error invalid detector type");
	}

	if(!info.classifier.load(path))
	{
		throw std::runtime_error("Error loading cascade data");
	}

	_featuremap[type].objects[name] = info;
}

void FeatureLibrary::SetTypeOverlayColor(FeatureType type, const cv::Scalar& color)
{
	_featuremap[type].overlayColor = color;
}

const cv::Scalar& FeatureLibrary::GetOverlayColor(FeatureType type) const
{
	return _featuremap.at(type).overlayColor;
}

const std::vector<cv::KeyPoint> &FeatureLibrary::GetKeypoints(FeatureType type, const std::string &name) const
{
	return _featuremap.at(type).objects.at(name).keypoints;
}

const cv::Mat& FeatureLibrary::GetDescriptors(FeatureType type, const std::string& name) const
{
	return _featuremap.at(type).objects.at(name).descriptor;
}

void FeatureLibrary::PreProcess(cv::Mat &image, cv::Mat& mat)
{
	switch(_type)
	{
	case DetectorType::SURFIllumCanny:
		PreProcessIllumCanny(image, mat);
		break;
	case DetectorType::Cascacade: 
		PreProcessCascade(image, mat);
		break;
	default: 
		mat = image;
		break;
	}
}

std::string FeatureLibrary::FindMatch(cv::Mat & image, double *avgDist, double *minDist, double *maxDist)
{
	double bestAvgDist = INFINITY;
	double bestMinDist = INFINITY;
	double bestMaxDist = 0;

	std::string bestMatch;
	for(auto type : _featuremap)
	{
		double curAvgDist, curMinDist, curMaxDist;
		std::string result = FindMatch(type.first, image, &curAvgDist, &curMinDist, &curMaxDist);
		if(!result.empty())
		{
			if (curAvgDist < bestAvgDist)
			{
				bestMatch = result;
				bestMinDist = curMinDist;
				bestMaxDist = curMaxDist;
				bestAvgDist = curAvgDist;
			}
		}
	}
	if (minDist) *minDist = bestMinDist;
	if (maxDist) *maxDist = bestMaxDist;
	if (avgDist) *avgDist = bestAvgDist;
	return bestMatch;
}

std::string FeatureLibrary::FindMatch(FeatureType type, cv::Mat & image, double *avgDist, double *minDist, double *maxDist)
{
	double bestAvgDist = INFINITY;
	double bestMinDist = INFINITY;
	double bestMaxDist = 0;
	std::string bestMatch;
	cv::Mat preprocessedImage;
	PreProcess(image, preprocessedImage);
	for (auto object : _featuremap.at(type).objects)
	{
		double curAvgDist, curMinDist, curMaxDist;
		if(FindMatch(type, object.first, preprocessedImage, &curAvgDist, &curMinDist, &curMaxDist))
		{
			if(curAvgDist < bestAvgDist)
			{
				bestMatch = object.first;
				bestMinDist = curMinDist;
				bestMaxDist = curMaxDist;
				bestAvgDist = curAvgDist;
			}
		}
	}
	if (minDist) *minDist = bestMinDist;
	if (maxDist) *maxDist = bestMaxDist;
	if (avgDist) *avgDist = bestAvgDist;
	return bestMatch;
}

bool FeatureLibrary::FindMatch(FeatureType type, const std::string name, cv::Mat & image, double *avgDist, double *minDist, double *maxDist)
{
	switch(_type)
	{
	case DetectorType::SURFIllumCanny:
	case DetectorType::SURF:
		return FindMatchSURFIllumCanny(type, name, image, avgDist, minDist, maxDist);
	case DetectorType::Cascacade:
		return FindMatchCascade(type, name, image, avgDist, minDist, maxDist);
	default:
		throw std::runtime_error("Invallid detector type");
	}
}

bool FeatureLibrary::FindMatchSURFIllumCanny(FeatureType type, const std::string name, cv::Mat & image, double* avgDist, double* minDist, double* maxDist)
{
	std::vector<cv::DMatch> matches;
	struct featureInfo &objectInfo = _featuremap.at(type).objects.at(name);

	struct featureInfo sceneInfo;
	_detector->detectAndCompute(image, cv::Mat(), sceneInfo.keypoints, sceneInfo.descriptor);

	if (sceneInfo.descriptor.rows == 0)
		return false;

	// Match descriptor 
	_matcher->match(objectInfo.descriptor, sceneInfo.descriptor, matches);

	// Find max and min distances
	double max_dist = 0;
	double min_dist = INFINITY;
	double avg_dist = 0;
	for (unsigned int i = 0; i < matches.size(); i++)
	{
		double dist = matches[i].distance;
		avg_dist += dist;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}
	avg_dist /= matches.size();

	// Find good matches
	std::vector<cv::DMatch> goodMatches;
	for (unsigned int i = 0; i < matches.size(); i++)
	{
		if (matches[i].distance <= 4 * min_dist)
		{
			goodMatches.push_back(matches[i]);
		}
	}

	cv::Mat img_matches;
	cv::drawMatches(objectInfo.image, objectInfo.keypoints, image, sceneInfo.keypoints, goodMatches, img_matches,
		cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	if (minDist) *minDist = min_dist;
	if (maxDist) *maxDist = max_dist;
	if (avgDist) *avgDist = avg_dist;
	return avg_dist <= 0.45;
}

bool FeatureLibrary::FindMatchCascade(FeatureType type, const std::string name, cv::Mat & image, double* avgDist, double* minDist, double* maxDist)
{
	struct featureInfo &objectInfo = _featuremap.at(type).objects.at(name);
	std::vector<cv::Rect> matches;
	cv::Size minSize(10,10);
	cv::Size maxSize(100,100);

	objectInfo.classifier.detectMultiScale(image, matches, 1.05, 0, 0, minSize, maxSize);

	return matches.size() > 0;
}

void FeatureLibrary::PreProcessIllumCanny(cv::Mat & in, cv::Mat& out)
{
	_clahe->apply(in, out);
	cv::Canny(out, out, 150, 200, 3);
}

void FeatureLibrary::PreProcessCascade(cv::Mat & in, cv::Mat& out)
{
	out = in;
}
