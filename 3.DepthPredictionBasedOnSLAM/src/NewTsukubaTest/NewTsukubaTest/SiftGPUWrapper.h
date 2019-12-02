#pragma once

#include <Windows.h>
#include <atlconv.h>
#include <GL\GL.h>
#include <string>
#include <vector>
#include <ppl.h>
#include <concurrent_vector.h>

#include "SiftGPU.h"
#include "VisualSLAMConfigure.h"

#define	MAX_COARSE_MATCH_COUNT	4096

class CSiftGPUWrapper
{
private:
	static	int				m_nCudaDeviceCount;
	/*static	boost::mutex	m_mutexSiftGPU;
	static	boost::mutex	m_mutexMatchGPU;*/
	static	bool	m_bIsBusy;
	static	HMODULE	m_hSiftGPUModule;
	static	SiftGPU*		m_pSiftGPU;
	static	SiftMatchGPU**	m_pMatchGPU;
	static	ComboSiftGPU*	m_pComboGPU;
	static	int				m_pCoarseMatchIdx[4096][2];
public:
	CSiftGPUWrapper();
	~CSiftGPUWrapper();

	static	bool	InitializeSiftGPU(int nCudaDeviceCount = 1);
	static	bool	InitializeSiftGPU(int nMaxImageWidth, int nMaxImageHeight, int nCudaDeviceCount = 1);
	static	bool	ReleaseSiftGPU(void);

	static	void	ExtractSiftFeature(char* pImageData, int nImageWidth, int nImageHeight, std::vector<SiftKeyPoint>& vKeyPoints, std::vector<float>& vDescriptors);
	static	void	FindCoarseMatch(const std::vector<float>& vSrcDescriptors, const std::vector<float>& vDstDescriptors, std::vector<std::pair<int, int>>& vCoarseMatchIdx, int nCudaDeviceIdx = 0);
	static	void	FindCoarseMatch(const std::vector<float>& vSrcDescriptors, const std::vector<float>& vDstDescriptors, Concurrency::concurrent_vector<std::pair<int, int>>& vCoarseMatchIdx, int nCudaDeviceIdx = 0);
	static	void	FindCoarseMatch(const std::vector<float>& vSrcDescriptors, const std::vector<float>& vDstDescriptors, std::vector<int>& vSrcToDstMatchIdx, int nCudaDeviceIdx = 0);

	static	void	ExtractSiftFeature(char* pImageData, int nImageWidth, int nImageHeight, std::vector<SiftKeyPoint>& vKeyPoints, std::vector<unsigned char>& vDescriptors);
	static	void	FindCoarseMatch(const std::vector<unsigned char>& vSrcDescriptors, const std::vector<unsigned char>& vDstDescriptors, std::vector<std::pair<int, int>>& vCoarseMatchIdx, int nCudaDeviceIdx = 0);
	static	void	FindCoarseMatch(const std::vector<unsigned char>& vSrcDescriptors, const std::vector<unsigned char>& vDstDescriptors, Concurrency::concurrent_vector<std::pair<int, int>>& vCoarseMatchIdx, int nCudaDeviceIdx = 0);
	static	void	FindCoarseMatch(const std::vector<unsigned char>& vSrcDescriptors, const std::vector<unsigned char>& vDstDescriptors, std::vector<int>& vSrcToDstMatchIdx, int nCudaDeviceIdx = 0);

	static	void	SetBusy(bool bIsBusy = true);
	static	bool	IsBusy(void);
};

