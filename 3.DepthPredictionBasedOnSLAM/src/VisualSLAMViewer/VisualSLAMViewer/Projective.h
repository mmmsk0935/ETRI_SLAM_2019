// Copyright (c) 2009, V. Lepetit, EPFL
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met: 

// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer. 
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution. 

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// The views and conclusions contained in the software and documentation are those
// of the authors and should not be interpreted as representing official policies, 
//   either expressed or implied, of the FreeBSD Project.

#pragma once

#include <random>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include "armadillo"

class epnp {
public:
	epnp(void);
	~epnp();

	std::vector<int> m_vInlierDataIndices;
	arma::mat K, R;
	arma::vec t;
	std::mt19937	en;
	int	m_nInlierDataCount;
	double	m_dbThreshold;
	arma::mat InitImagePoints, InitWorldPoints;

	void SetInitData(arma::mat& Intrinsic, arma::mat& ImagePoints, arma::mat& WorldPoints);
	void SetThreshold(double dbThreshold = 1.0);
	void EstimatePose(int nMaxRansacIteration = 500);

	void	SetSampleData(int* pSampleDataIdx);

	int ComputeInlierDataCount(double ER[3][3], double ET[3]);
	void ComputeInlierData(void);
	std::vector<int>& GetInlierDataIndices(void) {
		return m_vInlierDataIndices;
	}
	arma::mat& GetR(void) {
		return R;
	}
	arma::vec& GetT(void) {
		return t;
	}

	void set_internal_parameters(const double uc, const double vc, const double fu, const double fv);

	void set_maximum_number_of_correspondences(const int n);
	void reset_correspondences(void);
	void add_correspondence(const double X, const double Y, const double Z, const double u, const double v);

	double compute_pose(double R[3][3], double T[3]);

	void relative_error(double & rot_err, double & transl_err,
		const double Rtrue[3][3], const double ttrue[3],
		const double Rest[3][3], const double test[3]);

	void print_pose(const double R[3][3], const double t[3]);
	double reprojection_error(const double R[3][3], const double t[3]);

private:
	void choose_control_points(void);
	void compute_barycentric_coordinates(void);
	void fill_M(cv::Mat& M, const int row, const double * alphas, const double u, const double v);
	void compute_ccs(const double * betas, const double * ut);
	void compute_pcs(void);

	void solve_for_sign(void);

	void find_betas_approx_1(const cv::Mat& L_6x10, const cv::Mat& Rho, double * betas);
	void find_betas_approx_2(const cv::Mat& L_6x10, const cv::Mat& Rho, double * betas);
	void find_betas_approx_3(const cv::Mat& L_6x10, const cv::Mat& Rho, double * betas);
	void qr_solve(cv::Mat& A, cv::Mat& b, cv::Mat& X);

	double dot(const double * v1, const double * v2);
	double dist2(const double * p1, const double * p2);

	void compute_rho(double * rho);
	void compute_L_6x10(const double * ut, double * l_6x10);

	void gauss_newton(const cv::Mat& L_6x10, const cv::Mat& Rho, double current_betas[4]);
	void compute_A_and_b_gauss_newton(const double * l_6x10, const double * rho,
		double cb[4], cv::Mat& A, cv::Mat& b);

	double compute_R_and_t(const double * ut, const double * betas,
		double R[3][3], double t[3]);

	void estimate_R_and_t(double R[3][3], double t[3]);

	void copy_R_and_t(const double R_dst[3][3], const double t_dst[3],
		double R_src[3][3], double t_src[3]);

	void mat_to_quat(const double R[3][3], double q[4]);


	double uc, vc, fu, fv;

	double * pws, *us, *alphas, *pcs;
	int maximum_number_of_correspondences;
	int number_of_correspondences;

	double cws[4][3], ccs[4][3];
	double cws_determinant;
};