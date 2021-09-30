#ifndef YESTEREOCAMERA_HPP
#define YESTEREOCAMERA_HPP

#include <vector>
#include <string>

namespace SYE
{
	class YeStereoCamera
	{
	private:
	protected:
		virtual bool doCalibration(const char *pPath) = 0;
		virtual bool doCalibration(std::vector<std::string> &imgList) = 0;

	public:
		YeStereoCamera();
		//virtual ~YeStereoCamera();
	};

}

#endif
