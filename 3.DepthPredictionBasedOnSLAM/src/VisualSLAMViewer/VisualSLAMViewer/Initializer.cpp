#include "stdafx.h"
#include "Initializer.h"

CInitializer::CInitializer()
{
}

CInitializer::~CInitializer()
{
}

bool	CInitializer::Inialize(void)
{
	if (m_vInitialFrames.size() == 1) {
		///< marker based initialization
	}
	else {
		///< two frame based initialization
	}
	return true;
}
