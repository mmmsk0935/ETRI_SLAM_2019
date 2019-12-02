#pragma once

#include <iostream>
#include <vector>

#include "Frame.h"

class CInitializer
{
private:
	std::vector<CFrame>	m_vInitialFrames;
public:
	CInitializer();
	~CInitializer();

	bool	Inialize(void);
};