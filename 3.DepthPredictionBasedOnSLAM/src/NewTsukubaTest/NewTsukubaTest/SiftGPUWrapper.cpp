#include "stdafx.h"
#include "SiftGPUWrapper.h"

HMODULE			CSiftGPUWrapper::m_hSiftGPUModule = NULL;
SiftGPU*		CSiftGPUWrapper::m_pSiftGPU = NULL;
SiftMatchGPU**	CSiftGPUWrapper::m_pMatchGPU = NULL;
ComboSiftGPU*	CSiftGPUWrapper::m_pComboGPU = NULL;
int				CSiftGPUWrapper::m_pCoarseMatchIdx[4096][2];
//boost::mutex	CSiftGPUWrapper::m_mutexSiftGPU;
//boost::mutex	CSiftGPUWrapper::m_mutexMatchGPU;
bool			CSiftGPUWrapper::m_bIsBusy = false;
int				CSiftGPUWrapper::m_nCudaDeviceCount = 0;

////////////////////////////////////////////////////////////////////////////
#if !defined(SIFTGPU_STATIC) && !defined(SIFTGPU_DLL_RUNTIME) 
// SIFTGPU_STATIC comes from compiler
#define SIFTGPU_DLL_RUNTIME
// Load at runtime if the above macro defined
// comment the macro above to use static linking
#endif

////////////////////////////////////////////////////////////////////////////
// define REMOTE_SIFTGPU to run computation in multi-process (Or remote) mode
// in order to run on a remote machine, you need to start the server manually
// This mode allows you use Multi-GPUs by creating multiple servers
// #define REMOTE_SIFTGPU
// #define REMOTE_SERVER        NULL
// #define REMOTE_SERVER_PORT   7777


///////////////////////////////////////////////////////////////////////////
//#define DEBUG_SIFTGPU  //define this to use the debug version in windows

#ifdef _WIN32
#ifdef SIFTGPU_DLL_RUNTIME
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#define FREE_MYLIB FreeLibrary
#define GET_MYPROC GetProcAddress
#else
	//define this to get dll import definition for win32
#define SIFTGPU_DLL
#ifdef _DEBUG 
#pragma comment(lib, "siftgpu_d.lib")
#else
#pragma comment(lib, "siftgpu.lib")
#endif
#endif
#else
#ifdef SIFTGPU_DLL_RUNTIME
#include <dlfcn.h>
#define FREE_MYLIB dlclose
#define GET_MYPROC dlsym
#endif
#endif

CSiftGPUWrapper::CSiftGPUWrapper()
{
}

CSiftGPUWrapper::~CSiftGPUWrapper()
{
}

bool	CSiftGPUWrapper::InitializeSiftGPU(int nCudaDeviceCount)
{
	m_nCudaDeviceCount = nCudaDeviceCount;
	USES_CONVERSION;
#ifdef SIFTGPU_DLL_RUNTIME
#ifdef _WIN32
#ifdef _DEBUG
	m_hSiftGPUModule = LoadLibraryA("siftgpu_d.dll");
#else
	m_hSiftGPUModule = LoadLibraryA("siftgpu.dll");
#endif
#else
	void * hsiftgpu = dlopen("libsiftgpu.so", RTLD_LAZY);
#endif

	if (m_hSiftGPUModule == NULL) return 0;

#ifdef REMOTE_SIFTGPU
	ComboSiftGPU* (*pCreateRemoteSiftGPU) (int, char*) = NULL;
	pCreateRemoteSiftGPU = (ComboSiftGPU* (*) (int, char*)) GET_MYPROC(hsiftgpu, "CreateRemoteSiftGPU");
	m_pComboGPU = pCreateRemoteSiftGPU(REMOTE_SERVER_PORT, REMOTE_SERVER);
	m_pSiftGPU = m_pComboGPU;
	m_pMatchGPU = m_pComboGPU;
#else
	SiftGPU* (*pCreateNewSiftGPU)(int) = NULL;
	SiftMatchGPU* (*pCreateNewSiftMatchGPU)(int) = NULL;
	pCreateNewSiftGPU = (SiftGPU* (*) (int)) GET_MYPROC(m_hSiftGPUModule, "CreateNewSiftGPU");
	pCreateNewSiftMatchGPU = (SiftMatchGPU* (*)(int)) GET_MYPROC(m_hSiftGPUModule, "CreateNewSiftMatchGPU");
	m_pSiftGPU = pCreateNewSiftGPU(1);

	m_pMatchGPU = new SiftMatchGPU*[nCudaDeviceCount];
	for (int i = 0; i < nCudaDeviceCount; i++)
		m_pMatchGPU[i] = pCreateNewSiftMatchGPU(4096);
#endif

#elif defined(REMOTE_SIFTGPU)
	m_pComboGPU = CreateRemoteSiftGPU(REMOTE_SERVER_PORT, REMOTE_SERVER);
	m_pSiftGPU = m_pComboGPU;
	m_pMatchGPU = m_pComboGPU;
#else
	//this will use overloaded new operators
	m_pSiftGPU = new SiftGPU;
	m_pMatchGPU = new SiftMatchGPU(4096);
#endif

	//process parameters
	//The following parameters are default in V340
	//-m,       up to 2 orientations for each feature (change to single orientation by using -m 1)
	//-s        enable subpixel subscale (disable by using -s 0)

	//-fo -1    staring from -1 octave 
	//-v 1      only print out # feature and overall time
	//-loweo    add a (.5, .5) offset
	//-tc <num> set a soft limit to number of detected features

	//NEW:  parameters for  GPU-selection
	//1. CUDA.                   Use parameter "-cuda", "[device_id]"
	//2. OpenGL.				 Use "-Display", "display_name" to select monitor/GPU (XLIB/GLUT)
	//   		                 on windows the display name would be something like \\.\DISPLAY4

	//////////////////////////////////////////////////////////////////////////////////////
	//You use CUDA for nVidia graphic cards by specifying
	//-cuda   : cuda implementation (fastest for smaller images)
	//          CUDA-implementation allows you to create multiple instances for multiple threads
	//          Checkout src\TestWin\MultiThreadSIFT
	/////////////////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////////////////
	////////////////////////Two Important Parameters///////////////////////////
	// First, texture reallocation happens when image size increases, and too many 
	// reallocation may lead to allocatoin failure.  You should be careful when using 
	// siftgpu on a set of images with VARYING imag sizes. It is recommended that you 
	// preset the allocation size to the largest width and largest height by using function
	// AllocationPyramid or prameter '-p' (e.g. "-p", "1024x768").

	// Second, there is a parameter you may not be aware of: the allowed maximum working
	// dimension. All the SIFT octaves that needs a larger texture size will be skipped.
	// The default prameter is 2560 for the unpacked implementation and 3200 for the packed.
	// Those two default parameter is tuned to for 768MB of graphic memory. You should adjust
	// it for your own GPU memory. You can also use this to keep/skip the small featuers.
	// To change this, call function SetMaxDimension or use parameter "-maxd".
	//
	// NEW: by default SiftGPU will try to fit the cap of GPU memory, and reduce the working 
	// dimension so as to not allocate too much. This feature can be disabled by -nomc
	//////////////////////////////////////////////////////////////////////////////////////

	char *argv[] = { "-fo", "-1", "-v", "0" , "-cuda", "0" };//
	int argc = sizeof(argv) / sizeof(char*);

	m_pSiftGPU->ParseParam(argc, argv);

	if (m_pSiftGPU->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED) return false;

	for (int i = 0; i < nCudaDeviceCount; i++) {
		char pDeviceIdx[255];
		sprintf_s(pDeviceIdx, "%d", i);

		char* pArgvForMatch[] = { "-cuda", pDeviceIdx };

		m_pMatchGPU[i]->SetLanguage(SiftMatchGPU::SIFTMATCH_LANGUAGE::SIFTMATCH_CUDA);
		m_pMatchGPU[i]->SetDeviceParam(2, pArgvForMatch);
		m_pMatchGPU[i]->CreateContextGL();
	}

	return true;
}

bool	CSiftGPUWrapper::InitializeSiftGPU(int nMaxImageWidth, int nMaxImageHeight, int nCudaDeviceCount)
{
	m_nCudaDeviceCount = nCudaDeviceCount;
	USES_CONVERSION;
#ifdef SIFTGPU_DLL_RUNTIME
#ifdef _WIN32
#ifdef _DEBUG
	m_hSiftGPUModule = LoadLibrary(A2T("siftgpu_d.dll"));
#else
	m_hSiftGPUModule = LoadLibrary(A2T("siftgpu.dll"));
#endif
#else
	void * hsiftgpu = dlopen("libsiftgpu.so", RTLD_LAZY);
#endif

	if (m_hSiftGPUModule == NULL) return 0;

#ifdef REMOTE_SIFTGPU
	ComboSiftGPU* (*pCreateRemoteSiftGPU) (int, char*) = NULL;
	pCreateRemoteSiftGPU = (ComboSiftGPU* (*) (int, char*)) GET_MYPROC(hsiftgpu, "CreateRemoteSiftGPU");
	m_pComboGPU = pCreateRemoteSiftGPU(REMOTE_SERVER_PORT, REMOTE_SERVER);
	m_pSiftGPU = m_pComboGPU;
	m_pMatchGPU = m_pComboGPU;
#else
	SiftGPU* (*pCreateNewSiftGPU)(int) = NULL;
	SiftMatchGPU* (*pCreateNewSiftMatchGPU)(int) = NULL;
	pCreateNewSiftGPU = (SiftGPU* (*) (int)) GET_MYPROC(m_hSiftGPUModule, "CreateNewSiftGPU");
	pCreateNewSiftMatchGPU = (SiftMatchGPU* (*)(int)) GET_MYPROC(m_hSiftGPUModule, "CreateNewSiftMatchGPU");
	m_pSiftGPU = pCreateNewSiftGPU(1);

	m_pMatchGPU = new SiftMatchGPU*[nCudaDeviceCount];
	for (int i = 0; i < nCudaDeviceCount; i++)
		m_pMatchGPU[i] = pCreateNewSiftMatchGPU(4096);
#endif

#elif defined(REMOTE_SIFTGPU)
	m_pComboGPU = CreateRemoteSiftGPU(REMOTE_SERVER_PORT, REMOTE_SERVER);
	m_pSiftGPU = m_pComboGPU;
	m_pMatchGPU = m_pComboGPU;
#else
	//this will use overloaded new operators
	m_pSiftGPU = new SiftGPU;
	m_pMatchGPU = new SiftMatchGPU(4096);
#endif

	//process parameters
	//The following parameters are default in V340
	//-m,       up to 2 orientations for each feature (change to single orientation by using -m 1)
	//-s        enable subpixel subscale (disable by using -s 0)

	//-fo -1    staring from -1 octave 
	//-v 1      only print out # feature and overall time
	//-loweo    add a (.5, .5) offset
	//-tc <num> set a soft limit to number of detected features

	//NEW:  parameters for  GPU-selection
	//1. CUDA.                   Use parameter "-cuda", "[device_id]"
	//2. OpenGL.				 Use "-Display", "display_name" to select monitor/GPU (XLIB/GLUT)
	//   		                 on windows the display name would be something like \\.\DISPLAY4

	//////////////////////////////////////////////////////////////////////////////////////
	//You use CUDA for nVidia graphic cards by specifying
	//-cuda   : cuda implementation (fastest for smaller images)
	//          CUDA-implementation allows you to create multiple instances for multiple threads
	//          Checkout src\TestWin\MultiThreadSIFT
	/////////////////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////////////////
	////////////////////////Two Important Parameters///////////////////////////
	// First, texture reallocation happens when image size increases, and too many 
	// reallocation may lead to allocatoin failure.  You should be careful when using 
	// siftgpu on a set of images with VARYING imag sizes. It is recommended that you 
	// preset the allocation size to the largest width and largest height by using function
	// AllocationPyramid or prameter '-p' (e.g. "-p", "1024x768").

	// Second, there is a parameter you may not be aware of: the allowed maximum working
	// dimension. All the SIFT octaves that needs a larger texture size will be skipped.
	// The default prameter is 2560 for the unpacked implementation and 3200 for the packed.
	// Those two default parameter is tuned to for 768MB of graphic memory. You should adjust
	// it for your own GPU memory. You can also use this to keep/skip the small featuers.
	// To change this, call function SetMaxDimension or use parameter "-maxd".
	//
	// NEW: by default SiftGPU will try to fit the cap of GPU memory, and reduce the working 
	// dimension so as to not allocate too much. This feature can be disabled by -nomc
	//////////////////////////////////////////////////////////////////////////////////////

	char pImageSize[255];

	sprintf_s(pImageSize, sizeof(char) * 255, "%dx%d", nMaxImageWidth, nMaxImageHeight);

	char *argv[] = { "-fo", "-1", "-v", "0" , "-cuda", "0", "-p", pImageSize };//
	int argc = sizeof(argv) / sizeof(char*);

	m_pSiftGPU->ParseParam(argc, argv);

	if (m_pSiftGPU->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED) return false;

	for (int i = 0; i < nCudaDeviceCount; i++) {
		char pDeviceIdx[255];
		sprintf_s(pDeviceIdx, "%d", i);

		char* pArgvForMatch[] = { "-cuda", pDeviceIdx };

		m_pMatchGPU[i]->SetLanguage(SiftMatchGPU::SIFTMATCH_LANGUAGE::SIFTMATCH_CUDA);
		m_pMatchGPU[i]->SetDeviceParam(2, pArgvForMatch);
		m_pMatchGPU[i]->CreateContextGL();
	}

	return true;
}

bool	CSiftGPUWrapper::ReleaseSiftGPU(void)
{
#ifdef REMOTE_SIFTGPU
	delete m_pComboGPU;
#else
	delete m_pSiftGPU;

	for (int i = 0; i < m_nCudaDeviceCount; i++)
		delete m_pMatchGPU[i];
	delete[] m_pMatchGPU;

#endif

#ifdef SIFTGPU_DLL_RUNTIME
	FREE_MYLIB(m_hSiftGPUModule);
#endif

	m_pComboGPU = NULL;
	m_pSiftGPU = NULL;
	m_pMatchGPU = NULL;
	m_hSiftGPUModule = NULL;

	return true;
}

void	CSiftGPUWrapper::ExtractSiftFeature(char* pImageData, int nImageWidth, int nImageHeight, std::vector<SiftKeyPoint>& vKeyPoints, std::vector<float>& vDescriptors)
{
	//SetBusy(true);

	int nFeatureCount;

	//boost::mutex::scoped_lock lock(m_mutexSiftGPU);
	m_pSiftGPU->RunSIFT(nImageWidth, nImageHeight, pImageData, GL_LUMINANCE, GL_UNSIGNED_BYTE);

	nFeatureCount = m_pSiftGPU->GetFeatureNum();

	vKeyPoints.resize(nFeatureCount);
	vDescriptors.resize(nFeatureCount * 128);

	if (nFeatureCount == 0)		return;

	m_pSiftGPU->GetFeatureVector((SiftGPU::SiftKeypoint *)&vKeyPoints[0], &vDescriptors[0]);

	//SetBusy(false);
}

//void	CSiftGPUWrapper::ExtractSiftFeature(char* pImageData, int nImageWidth, int nImageHeight, std::vector<SiftKeyPoint>& vKeyPoints, std::vector<unsigned char>& vDescriptors)
//{
//	SetBusy(true);
//
//	int nFeatureCount;
//	std::vector<float> vTmpDescriptors;
//	m_pSiftGPU->RunSIFT(nImageWidth, nImageHeight, pImageData, GL_LUMINANCE, GL_UNSIGNED_BYTE);
//
//	nFeatureCount = m_pSiftGPU->GetFeatureNum();
//
//	vKeyPoints.resize(nFeatureCount);
//	vTmpDescriptors.resize(nFeatureCount * 128);
//
//	if (nFeatureCount == 0)		return;
//
//	m_pSiftGPU->GetFeatureVector((SiftGPU::SiftKeypoint *)&vKeyPoints[0], &vTmpDescriptors[0]);
//
//	vDescriptors.resize(nFeatureCount * 128);
//
//	Concurrency::parallel_for(0, nFeatureCount, [&](int i) {
//		for (int j = 0; j < 128; j++)
//			vDescriptors[i * 128 + j] = (unsigned char)(vTmpDescriptors[i * 128 + j] * 512 + 0.5);
//	});
//
//	SetBusy(false);
//}

void	CSiftGPUWrapper::FindCoarseMatch(const std::vector<float>& vSrcDescriptors, const std::vector<float>& vDstDescriptors, std::vector<std::pair<int, int>>& vCoarseMatchIdx, int nCudaDeviceIdx)
{
	//SetBusy(true);

	int nSrcFeatureCount = (int)vSrcDescriptors.size() / 128;
	int nDstFeatureCount = (int)vDstDescriptors.size() / 128;

	if (nSrcFeatureCount == 0 || nDstFeatureCount == 0) {
		vCoarseMatchIdx.clear();
		return;
	}

	int nMaxFeatureCount = (nSrcFeatureCount > nDstFeatureCount) ? nSrcFeatureCount : nDstFeatureCount;
	int nMaxCoarseMatchCount = (MAX_COARSE_MATCH_COUNT < nMaxFeatureCount) ? MAX_COARSE_MATCH_COUNT : nMaxFeatureCount;

	m_pMatchGPU[nCudaDeviceIdx]->SetDescriptors(0, nSrcFeatureCount, &vSrcDescriptors[0]);
	m_pMatchGPU[nCudaDeviceIdx]->SetDescriptors(1, nDstFeatureCount, &vDstDescriptors[0]);

	int nCoarseMatchCount = m_pMatchGPU[nCudaDeviceIdx]->GetSiftMatch(nMaxCoarseMatchCount, m_pCoarseMatchIdx);

	vCoarseMatchIdx.resize(nCoarseMatchCount);
	Concurrency::parallel_for(0, nCoarseMatchCount, [&](int i) {
		vCoarseMatchIdx[i].first = m_pCoarseMatchIdx[i][0];
		vCoarseMatchIdx[i].second = m_pCoarseMatchIdx[i][1];
	});

	//SetBusy(false);
}

void	CSiftGPUWrapper::FindCoarseMatch(const std::vector<float>& vSrcDescriptors, const std::vector<float>& vDstDescriptors, Concurrency::concurrent_vector<std::pair<int, int>>& vCoarseMatchIdx, int nCudaDeviceIdx)
{
	//SetBusy(true);

	int nSrcFeatureCount = (int)vSrcDescriptors.size() / 128;
	int nDstFeatureCount = (int)vDstDescriptors.size() / 128;

	if (nSrcFeatureCount == 0 || nDstFeatureCount == 0) {
		vCoarseMatchIdx.clear();
		return;
	}

	int nMaxFeatureCount = (nSrcFeatureCount > nDstFeatureCount) ? nSrcFeatureCount : nDstFeatureCount;
	int nMaxCoarseMatchCount = (MAX_COARSE_MATCH_COUNT < nMaxFeatureCount) ? MAX_COARSE_MATCH_COUNT : nMaxFeatureCount;
	m_pMatchGPU[nCudaDeviceIdx]->SetDescriptors(0, nSrcFeatureCount, &vSrcDescriptors[0]);
	m_pMatchGPU[nCudaDeviceIdx]->SetDescriptors(1, nDstFeatureCount, &vDstDescriptors[0]);

	int nCoarseMatchCount = m_pMatchGPU[nCudaDeviceIdx]->GetSiftMatch(nMaxCoarseMatchCount, m_pCoarseMatchIdx);

	vCoarseMatchIdx.resize(nCoarseMatchCount);
	Concurrency::parallel_for(0, nCoarseMatchCount, [&](int i) {
		vCoarseMatchIdx[i].first = m_pCoarseMatchIdx[i][0];
		vCoarseMatchIdx[i].second = m_pCoarseMatchIdx[i][1];
	});

	//SetBusy(false);
}

void	CSiftGPUWrapper::FindCoarseMatch(const std::vector<float>& vSrcDescriptors, const std::vector<float>& vDstDescriptors, std::vector<int>& vSrcToDstMatchIdx, int nCudaDeviceIdx)
{
	//SetBusy(true);

	int nSrcFeatureCount = (int)vSrcDescriptors.size() / 128;
	int nDstFeatureCount = (int)vDstDescriptors.size() / 128;

	if (nSrcFeatureCount == 0 || nDstFeatureCount == 0) {
		vSrcToDstMatchIdx.clear();
		return;
	}

	int nMaxFeatureCount = (nSrcFeatureCount > nDstFeatureCount) ? nSrcFeatureCount : nDstFeatureCount;
	int nMaxCoarseMatchCount = (MAX_COARSE_MATCH_COUNT < nMaxFeatureCount) ? MAX_COARSE_MATCH_COUNT : nMaxFeatureCount;
	m_pMatchGPU[nCudaDeviceIdx]->SetDescriptors(0, nSrcFeatureCount, &vSrcDescriptors[0]);
	m_pMatchGPU[nCudaDeviceIdx]->SetDescriptors(1, nDstFeatureCount, &vDstDescriptors[0]);

	int nCoarseMatchCount = m_pMatchGPU[nCudaDeviceIdx]->GetSiftMatch(nMaxCoarseMatchCount, m_pCoarseMatchIdx);

	vSrcToDstMatchIdx.resize(nSrcFeatureCount);
	memset(&vSrcToDstMatchIdx[0], -1, sizeof(int) * nSrcFeatureCount);

	Concurrency::parallel_for(0, nCoarseMatchCount, [&](int i) {
		vSrcToDstMatchIdx[m_pCoarseMatchIdx[i][0]] = m_pCoarseMatchIdx[i][1];
	});

	//SetBusy(false);
}

void	CSiftGPUWrapper::ExtractSiftFeature(char* pImageData, int nImageWidth, int nImageHeight, std::vector<SiftKeyPoint>& vKeyPoints, std::vector<unsigned char>& vDescriptors)
{
	//SetBusy(true);

	int nFeatureCount;
	std::vector<float> vTmpDescriptors;

	m_pSiftGPU->RunSIFT(nImageWidth, nImageHeight, pImageData, GL_LUMINANCE, GL_UNSIGNED_BYTE);

	nFeatureCount = m_pSiftGPU->GetFeatureNum();

	vKeyPoints.resize(nFeatureCount);
	vDescriptors.resize(nFeatureCount * 128);
	vTmpDescriptors.resize(nFeatureCount * 128);

	if (nFeatureCount == 0)		return;

	m_pSiftGPU->GetFeatureVector((SiftGPU::SiftKeypoint *)&vKeyPoints[0], &vTmpDescriptors[0]);

	Concurrency::parallel_for(0, nFeatureCount, [&](int i) {
		float* pTmpDescriptors = &vTmpDescriptors[i * 128];
		unsigned char* pDescriptors = &vDescriptors[i * 128];
		for (int j = 0; j < 128; j++)
			pDescriptors[j] = (unsigned char)(pTmpDescriptors[j] * 512 + 0.5);
	});

	//SetBusy(false);
}

void	CSiftGPUWrapper::FindCoarseMatch(const std::vector<unsigned char>& vSrcDescriptors, const std::vector<unsigned char>& vDstDescriptors, std::vector<std::pair<int, int>>& vCoarseMatchIdx, int nCudaDeviceIdx)
{
	//SetBusy(true);

	int nSrcFeatureCount = (int)vSrcDescriptors.size() / 128;
	int nDstFeatureCount = (int)vDstDescriptors.size() / 128;

	if (nSrcFeatureCount == 0 || nDstFeatureCount == 0) {
		vCoarseMatchIdx.clear();
		return;
	}

	int nMaxFeatureCount = (nSrcFeatureCount > nDstFeatureCount) ? nSrcFeatureCount : nDstFeatureCount;
	int nMaxCoarseMatchCount = (MAX_COARSE_MATCH_COUNT < nMaxFeatureCount) ? MAX_COARSE_MATCH_COUNT : nMaxFeatureCount;
	m_pMatchGPU[nCudaDeviceIdx]->SetDescriptors(0, nSrcFeatureCount, &vSrcDescriptors[0]);
	m_pMatchGPU[nCudaDeviceIdx]->SetDescriptors(1, nDstFeatureCount, &vDstDescriptors[0]);

	int nCoarseMatchCount = m_pMatchGPU[nCudaDeviceIdx]->GetSiftMatch(nMaxCoarseMatchCount, m_pCoarseMatchIdx);

	vCoarseMatchIdx.resize(nCoarseMatchCount);
	Concurrency::parallel_for(0, nCoarseMatchCount, [&](int i) {
		vCoarseMatchIdx[i].first = m_pCoarseMatchIdx[i][0];
		vCoarseMatchIdx[i].second = m_pCoarseMatchIdx[i][1];
	});

	//SetBusy(false);
}

void	CSiftGPUWrapper::FindCoarseMatch(const std::vector<unsigned char>& vSrcDescriptors, const std::vector<unsigned char>& vDstDescriptors, Concurrency::concurrent_vector<std::pair<int, int>>& vCoarseMatchIdx, int nCudaDeviceIdx)
{
	//SetBusy(true);

	int nSrcFeatureCount = (int)vSrcDescriptors.size() / 128;
	int nDstFeatureCount = (int)vDstDescriptors.size() / 128;

	if (nSrcFeatureCount == 0 || nDstFeatureCount == 0) {
		vCoarseMatchIdx.clear();
		return;
	}

	int nMaxFeatureCount = (nSrcFeatureCount > nDstFeatureCount) ? nSrcFeatureCount : nDstFeatureCount;
	int nMaxCoarseMatchCount = (MAX_COARSE_MATCH_COUNT < nMaxFeatureCount) ? MAX_COARSE_MATCH_COUNT : nMaxFeatureCount;
	m_pMatchGPU[nCudaDeviceIdx]->SetDescriptors(0, nSrcFeatureCount, &vSrcDescriptors[0]);
	m_pMatchGPU[nCudaDeviceIdx]->SetDescriptors(1, nDstFeatureCount, &vDstDescriptors[0]);

	int nCoarseMatchCount = m_pMatchGPU[nCudaDeviceIdx]->GetSiftMatch(nMaxCoarseMatchCount, m_pCoarseMatchIdx);

	vCoarseMatchIdx.resize(nCoarseMatchCount);
	Concurrency::parallel_for(0, nCoarseMatchCount, [&](int i) {
		vCoarseMatchIdx[i].first = m_pCoarseMatchIdx[i][0];
		vCoarseMatchIdx[i].second = m_pCoarseMatchIdx[i][1];
	});

	//SetBusy(false);
}

void	CSiftGPUWrapper::FindCoarseMatch(const std::vector<unsigned char>& vSrcDescriptors, const std::vector<unsigned char>& vDstDescriptors, std::vector<int>& vSrcToDstMatchIdx, int nCudaDeviceIdx)
{
	//SetBusy(true);

	int nSrcFeatureCount = (int)vSrcDescriptors.size() / 128;
	int nDstFeatureCount = (int)vDstDescriptors.size() / 128;

	if (nSrcFeatureCount == 0 || nDstFeatureCount == 0) {
		vSrcToDstMatchIdx.clear();
		return;
	}

	int nMaxFeatureCount = (nSrcFeatureCount > nDstFeatureCount) ? nSrcFeatureCount : nDstFeatureCount;
	int nMaxCoarseMatchCount = (MAX_COARSE_MATCH_COUNT < nMaxFeatureCount) ? MAX_COARSE_MATCH_COUNT : nMaxFeatureCount;

	m_pMatchGPU[nCudaDeviceIdx]->SetDescriptors(0, nSrcFeatureCount, &vSrcDescriptors[0]);
	m_pMatchGPU[nCudaDeviceIdx]->SetDescriptors(1, nDstFeatureCount, &vDstDescriptors[0]);

	int nCoarseMatchCount = m_pMatchGPU[nCudaDeviceIdx]->GetSiftMatch(nMaxCoarseMatchCount, m_pCoarseMatchIdx);

	vSrcToDstMatchIdx.resize(nSrcFeatureCount);
	memset(&vSrcToDstMatchIdx[0], -1, sizeof(int) * nSrcFeatureCount);

	Concurrency::parallel_for(0, nCoarseMatchCount, [&](int i) {
		vSrcToDstMatchIdx[m_pCoarseMatchIdx[i][0]] = m_pCoarseMatchIdx[i][1];
	});

	//SetBusy(false);
}

void	CSiftGPUWrapper::SetBusy(bool bIsBusy/* = true*/)
{
	//	boost::mutex::scoped_lock lock(m_mutexBusy);
	m_bIsBusy = bIsBusy;
}

bool	CSiftGPUWrapper::IsBusy(void)
{
	//boost::mutex::scoped_lock lock(m_mutexBusy);
	return m_bIsBusy;
}