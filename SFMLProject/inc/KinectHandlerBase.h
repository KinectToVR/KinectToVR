#pragma once
#include "IKinectHandler.h"
#include <opencv2/opencv.hpp>

class KinectHandlerBase : public IKinectHandler
{
public:
	KinectHandlerBase()
	{
	}

	~KinectHandlerBase()
	{
	}

	bool convertColorToDepthResolution = false;
	// Color Buffer
	std::vector<BYTE> colorBuffer;
	int colorWidth;
	int colorHeight;
	unsigned int colorBytesPerPixel;
	cv::Mat colorMat;

	// Depth Buffer
	std::vector<UINT16> depthBuffer;
	int depthWidth;
	int depthHeight;
	unsigned int depthBytesPerPixel;
	cv::Mat depthMat;

	void initOpenGL() override
	{
	};

	void initialise() override
	{
	};

	virtual void initialiseSkeleton()
	{
	};

	virtual void initialiseColor()
	{
	};

	virtual void initialiseDepth()
	{
	};

	virtual void terminateSkeleton()
	{
	};

	virtual void terminateColor()
	{
	};

	virtual void terminateDepth()
	{
	};

	HRESULT getStatusResult() override { return E_NOTIMPL; }
	std::string statusResultString(HRESULT stat) override { return "statusResultString behaviour not defined"; };
	
	void update() override
	{
	};
	
	void drawKinectData(sf::RenderWindow& win) override
	{
	}; // Houses the below draw functions with a check
	void drawKinectImageData(sf::RenderWindow& win) override
	{
	};

	void drawTrackedSkeletons(sf::RenderWindow& win) override
	{
	}
};
