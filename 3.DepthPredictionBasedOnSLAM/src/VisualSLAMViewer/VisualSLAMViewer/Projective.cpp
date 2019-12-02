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

#include "stdafx.h"
#include <iostream>
using namespace std;

#include "projective.h"

epnp::epnp(void)
{
	maximum_number_of_correspondences = 0;
	number_of_correspondences = 0;

	pws = 0;
	us = 0;
	alphas = 0;
	pcs = 0;
}

epnp::~epnp()
{
	delete[] pws;
	delete[] us;
	delete[] alphas;
	delete[] pcs;
}

void epnp::SetInitData(arma::mat& Intrinsic, arma::mat& ImagePoints, arma::mat& WorldPoints)
{
	K = Intrinsic;

	fu = K.at(0, 0);
	fv = K.at(1, 1);
	uc = K.at(0, 2);
	vc = K.at(1, 2);

	InitImagePoints = ImagePoints;

	InitWorldPoints.set_size(4, WorldPoints.n_cols);

	InitWorldPoints(arma::span(0, 2), arma::span(0, WorldPoints.n_cols-1)) = WorldPoints;
	InitWorldPoints.row(3).fill(1.0);

	set_maximum_number_of_correspondences(4);	///< minimum set for epnp

	R.eye(3, 3);
	t.zeros(3);
}

void epnp::SetThreshold(double dbThreshold/* = 1.0*/)
{
	m_dbThreshold = dbThreshold;
}

void epnp::EstimatePose(int nMaxRansacIteration/* = 500*/)
{
	int nInlierDataCount;
	
	m_nInlierDataCount = 0;
	for (int i = 0; i < nMaxRansacIteration; i++) {
		int pSampleDataIdx[4];
		double ER[3][3], ET[3];

		SetSampleData(pSampleDataIdx);

		reset_correspondences();
		for (int j = 0; j < 4; j++) {
			add_correspondence(InitWorldPoints.at(0, pSampleDataIdx[j]), 
							   InitWorldPoints.at(1, pSampleDataIdx[j]), 
							   InitWorldPoints.at(2, pSampleDataIdx[j]), 
							   InitImagePoints.at(0, pSampleDataIdx[j]), 
							   InitImagePoints.at(1, pSampleDataIdx[j]));
		}

		compute_pose(ER, ET);

		nInlierDataCount = ComputeInlierDataCount(ER, ET);

		if (nInlierDataCount > m_nInlierDataCount) {
			m_nInlierDataCount = nInlierDataCount;
			for (int row = 0; row < 3; row++) {
				for (int col = 0; col < 3; col++) {
					R.at(row, col) = ER[row][col];
				}
				t.at(row) = ET[row];
			}
		}
	}

	ComputeInlierData();
}

void epnp::SetSampleData(int *pSampleDataIdx)
{
	bool bDuplicate;
	int nRandomIdx;

	std::uniform_int_distribution<int> uid(0, (int)InitWorldPoints.n_cols - 1);

	for (int i = 0; i < 4; i++) {
		bDuplicate = false;
		nRandomIdx = uid(en);

		for (int j = 0; j < i; j++) {
			if (pSampleDataIdx[j] == nRandomIdx) {
				bDuplicate = true;
				break;
			}
		}

		if (bDuplicate)	i--;
		else			pSampleDataIdx[i] = nRandomIdx;
	}
}

int epnp::ComputeInlierDataCount(double ER[3][3], double ET[3])
{
	int nInlierDataCount = 0;
	arma::mat R1(3, 3), P1(3, 4);
	arma::vec t1(3);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			R1.at(i, j) = ER[i][j];
		}
		t1.at(i) = ET[i];
	}

	P1(arma::span(0, 2), arma::span(0, 2)) = R1;
	P1(arma::span(0, 2), arma::span(3, 3)) = t1;
	P1 = K * P1;

	arma::mat WorldToImage = P1 * InitWorldPoints;
	for (int i = 0; i < (int)WorldToImage.n_cols; i++)
		WorldToImage.col(i) /= WorldToImage.at(2, i);

	arma::mat Diff = InitImagePoints - WorldToImage;
	arma::rowvec Dist = arma::sum(Diff%Diff);

	for (int i = 0; i < (int)InitWorldPoints.n_cols; i++) {
		if (Dist.at(i) < m_dbThreshold)
			nInlierDataCount++;
	}

	return nInlierDataCount;
}

void epnp::ComputeInlierData(void)
{
	arma::mat P(3, 4);

	P(arma::span(0, 2), arma::span(0, 2)) = R;
	P(arma::span(0, 2), arma::span(3, 3)) = t;
	P = K * P;

	arma::mat WorldToImage = P * InitWorldPoints;
	for (int i = 0; i < (int)WorldToImage.n_cols; i++)
		WorldToImage.col(i) /= WorldToImage.at(2, i);

	arma::mat Diff = InitImagePoints - WorldToImage;
	arma::rowvec Dist = arma::sum(Diff%Diff);

	m_vInlierDataIndices.clear();
	for (int i = 0; i < (int)InitWorldPoints.n_cols; i++) {
		if (Dist.at(i) < m_dbThreshold)
			m_vInlierDataIndices.push_back(i);
	}

	assert((int)m_vInlierDataIndices.size() == m_nInlierDataCount);
}

void epnp::set_internal_parameters(double uc, double vc, double fu, double fv)
{
	this->uc = uc;
	this->vc = vc;
	this->fu = fu;
	this->fv = fv;
}

void epnp::set_maximum_number_of_correspondences(int n)
{
	if (maximum_number_of_correspondences < n) {
		if (pws != 0) delete[] pws;
		if (us != 0) delete[] us;
		if (alphas != 0) delete[] alphas;
		if (pcs != 0) delete[] pcs;

		maximum_number_of_correspondences = n;
		pws = new double[3 * maximum_number_of_correspondences];
		us = new double[2 * maximum_number_of_correspondences];
		alphas = new double[4 * maximum_number_of_correspondences];
		pcs = new double[3 * maximum_number_of_correspondences];
	}
}

void epnp::reset_correspondences(void)
{
	number_of_correspondences = 0;
}

void epnp::add_correspondence(double X, double Y, double Z, double u, double v)
{
	pws[3 * number_of_correspondences] = X;
	pws[3 * number_of_correspondences + 1] = Y;
	pws[3 * number_of_correspondences + 2] = Z;

	us[2 * number_of_correspondences] = u;
	us[2 * number_of_correspondences + 1] = v;

	number_of_correspondences++;
}

void epnp::choose_control_points(void)
{
	// Take C0 as the reference points centroid:
	cws[0][0] = cws[0][1] = cws[0][2] = 0;
	for (int i = 0; i < number_of_correspondences; i++)
		for (int j = 0; j < 3; j++)
			cws[0][j] += pws[3 * i + j];

	for (int j = 0; j < 3; j++)
		cws[0][j] /= number_of_correspondences;


	// Take C1, C2, and C3 from PCA on the reference points:
	cv::Mat& PW0 = cv::Mat(number_of_correspondences, 3, CV_64F);

	/*double pw0tpw0[3 * 3], dc[3], uct[3 * 3];
	cv::Mat PW0tPW0 = cv::Mat(3, 3, CV_64F, pw0tpw0);
	cv::Mat DC = cv::Mat(3, 1, CV_64F, dc);
	cv::Mat UCt = cv::Mat(3, 3, CV_64F, uct);*/

	//double pw0tpw0[3 * 3], dc[3], uct[3 * 3];
	cv::Mat PW0tPW0 = cv::Mat(3, 3, CV_64F);
	cv::Mat DC = cv::Mat(3, 1, CV_64F);
	cv::Mat UCt = cv::Mat(3, 3, CV_64F);
	
	for (int i = 0; i < number_of_correspondences; i++)
		for (int j = 0; j < 3; j++)
			PW0.at<double>(i, j) = pws[3 * i + j] - cws[0][j];

	PW0tPW0 = PW0.t() * PW0;
	//cvMulTransposed(PW0, &PW0tPW0, 1);
	cv::SVD svd(PW0tPW0, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
	DC = svd.w;
	UCt = svd.u.t();
	//cvSVD(&PW0tPW0, &DC, &UCt, 0, CV_SVD_MODIFY_A | CV_SVD_U_T);

	//cvReleaseMat(&PW0);
	double *pw0tpw0 = (double*)PW0tPW0.data;
	double *dc = (double*)DC.data;
	double *uct = (double*)UCt.data;

	for (int i = 1; i < 4; i++) {
		//double k = sqrt(dc[i - 1] / number_of_correspondences);
		double k = sqrt(DC.at<double>(i-1) / number_of_correspondences);
		for (int j = 0; j < 3; j++)
			cws[i][j] = cws[0][j] + k * UCt.at<double>(i - 1, j);
			//cws[i][j] = cws[0][j] + k * uct[3 * (i - 1) + j];
	}
}

void epnp::compute_barycentric_coordinates(void)
{
	//double cc[3 * 3] , cc_inv[3 * 3];
	//cv::Mat CC = cv::Mat(3, 3, CV_64F, cc);
	cv::Mat CC = cv::Mat(3, 3, CV_64F);
	cv::Mat CC_inv = cv::Mat(3, 3, CV_64F);
	//cv::Mat CC_inv = cv::Mat(3, 3, CV_64F, cc_inv);

	double* cc = (double*)CC.data;
	for (int i = 0; i < 3; i++)
		for (int j = 1; j < 4; j++)
			cc[3 * i + j - 1] = cws[j][i] - cws[0][i];

	CC_inv = CC.inv(cv::DECOMP_SVD);
	//cvInvert(&CC, &CC_inv, CV_SVD);
	double * ci = (double*)CC_inv.data;
	for (int i = 0; i < number_of_correspondences; i++) {
		double * pi = pws + 3 * i;
		double * a = alphas + 4 * i;

		for (int j = 0; j < 3; j++)
			a[1 + j] =
			ci[3 * j    ] * (pi[0] - cws[0][0]) +
			ci[3 * j + 1] * (pi[1] - cws[0][1]) +
			ci[3 * j + 2] * (pi[2] - cws[0][2]);
		a[0] = 1.0f - a[1] - a[2] - a[3];
	}
}

void epnp::fill_M(cv::Mat& M, const int row, const double * as, const double u, const double v)
{
	//double * M1 = M->data.db + row * 12;
	double * M1 = (double*)M.data + row * 12;
	double * M2 = M1 + 12;

	for (int i = 0; i < 4; i++) {
		M1[3 * i    ] = as[i] * fu;
		M1[3 * i + 1] = 0.0;
		M1[3 * i + 2] = as[i] * (uc - u);

		M2[3 * i] = 0.0;
		M2[3 * i + 1] = as[i] * fv;
		M2[3 * i + 2] = as[i] * (vc - v);
	}
}

void epnp::compute_ccs(const double * betas, const double * ut)
{
	for (int i = 0; i < 4; i++)
		ccs[i][0] = ccs[i][1] = ccs[i][2] = 0.0f;

	for (int i = 0; i < 4; i++) {
		const double * v = ut + 12 * (11 - i);
		for (int j = 0; j < 4; j++)
			for (int k = 0; k < 3; k++)
				ccs[j][k] += betas[i] * v[3 * j + k];
	}
}

void epnp::compute_pcs(void)
{
	for (int i = 0; i < number_of_correspondences; i++) {
		double * a = alphas + 4 * i;
		double * pc = pcs + 3 * i;

		for (int j = 0; j < 3; j++)
			pc[j] = a[0] * ccs[0][j] + a[1] * ccs[1][j] + a[2] * ccs[2][j] + a[3] * ccs[3][j];
	}
}

double epnp::compute_pose(double R[3][3], double t[3])
{
	choose_control_points();
	compute_barycentric_coordinates();

	cv::Mat M = cv::Mat(2 * number_of_correspondences, 12, CV_64F);

	for (int i = 0; i < number_of_correspondences; i++)
		fill_M(M, 2 * i, alphas + 4 * i, us[2 * i], us[2 * i + 1]);

	/*double mtm[12 * 12], d[12], ut[12 * 12];
	cv::Mat MtM = cv::Mat(12, 12, CV_64F, mtm);
	cv::Mat D = cv::Mat(12, 1, CV_64F, d);
	cv::Mat Ut = cv::Mat(12, 12, CV_64F, ut);*/
	
	cv::Mat MtM = cv::Mat(12, 12, CV_64F);
	cv::Mat D = cv::Mat(12, 1, CV_64F);
	cv::Mat Ut = cv::Mat(12, 12, CV_64F);

	MtM = M.t() * M;
	//cvMulTransposed(M, &MtM, 1);
	cv::SVD svd(MtM, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
	D = svd.w;
	Ut = svd.u.t();
	//cvSVD(&MtM, &D, &Ut, 0, CV_SVD_MODIFY_A | CV_SVD_U_T);
	//cvReleaseMat(&M);

	double* mtm = (double*)MtM.data;
	double* d = (double*)D.data;
	double* ut = (double*)Ut.data;

	/*double l_6x10[6 * 10], rho[6];
	cv::Mat L_6x10 = cv::Mat(6, 10, CV_64F, l_6x10);
	cv::Mat Rho = cv::Mat(6, 1, CV_64F, rho);*/

	cv::Mat L_6x10 = cv::Mat(6, 10, CV_64F);
	cv::Mat Rho = cv::Mat(6, 1, CV_64F);

	double* l_6x10 = (double*)L_6x10.data;
	double* rho = (double*)Rho.data;

	compute_L_6x10(ut, l_6x10);
	compute_rho(rho);

	double Betas[4][4], rep_errors[4];
	double Rs[4][3][3], ts[4][3];

	find_betas_approx_1(L_6x10, Rho, Betas[1]);
	gauss_newton(L_6x10, Rho, Betas[1]);
	rep_errors[1] = compute_R_and_t(ut, Betas[1], Rs[1], ts[1]);

	find_betas_approx_2(L_6x10, Rho, Betas[2]);
	gauss_newton(L_6x10, Rho, Betas[2]);
	rep_errors[2] = compute_R_and_t(ut, Betas[2], Rs[2], ts[2]);

	find_betas_approx_3(L_6x10, Rho, Betas[3]);
	gauss_newton(L_6x10, Rho, Betas[3]);
	rep_errors[3] = compute_R_and_t(ut, Betas[3], Rs[3], ts[3]);

	int N = 1;
	if (rep_errors[2] < rep_errors[1]) N = 2;
	if (rep_errors[3] < rep_errors[N]) N = 3;

	copy_R_and_t(Rs[N], ts[N], R, t);

	return rep_errors[N];
}

void epnp::copy_R_and_t(const double R_src[3][3], const double t_src[3], double R_dst[3][3], double t_dst[3])
{
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++)
			R_dst[i][j] = R_src[i][j];
		t_dst[i] = t_src[i];
	}
}

double epnp::dist2(const double * p1, const double * p2)
{
	return
		(p1[0] - p2[0]) * (p1[0] - p2[0]) +
		(p1[1] - p2[1]) * (p1[1] - p2[1]) +
		(p1[2] - p2[2]) * (p1[2] - p2[2]);
}

double epnp::dot(const double * v1, const double * v2)
{
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

double epnp::reprojection_error(const double R[3][3], const double t[3])
{
	double sum2 = 0.0;

	for (int i = 0; i < number_of_correspondences; i++) {
		double * pw = pws + 3 * i;
		double Xc = dot(R[0], pw) + t[0];
		double Yc = dot(R[1], pw) + t[1];
		double inv_Zc = 1.0 / (dot(R[2], pw) + t[2]);
		double ue = uc + fu * Xc * inv_Zc;
		double ve = vc + fv * Yc * inv_Zc;
		double u = us[2 * i], v = us[2 * i + 1];

		sum2 += sqrt((u - ue) * (u - ue) + (v - ve) * (v - ve));
	}

	return sum2 / number_of_correspondences;
}

void epnp::estimate_R_and_t(double R[3][3], double t[3])
{
	double pc0[3], pw0[3];

	pc0[0] = pc0[1] = pc0[2] = 0.0;
	pw0[0] = pw0[1] = pw0[2] = 0.0;

	for (int i = 0; i < number_of_correspondences; i++) {
		const double * pc = pcs + 3 * i;
		const double * pw = pws + 3 * i;

		for (int j = 0; j < 3; j++) {
			pc0[j] += pc[j];
			pw0[j] += pw[j];
		}
	}
	for (int j = 0; j < 3; j++) {
		pc0[j] /= number_of_correspondences;
		pw0[j] /= number_of_correspondences;
	}

	/*double abt[3 * 3] = { 0 }, abt_d[3], abt_u[3 * 3], abt_v[3 * 3];
	cv::Mat ABt = cv::Mat(3, 3, CV_64F, abt);
	cv::Mat ABt_D = cv::Mat(3, 1, CV_64F, abt_d);
	cv::Mat ABt_U = cv::Mat(3, 3, CV_64F, abt_u);
	cv::Mat ABt_V = cv::Mat(3, 3, CV_64F, abt_v);*/

	cv::Mat ABt = cv::Mat::zeros(3, 3, CV_64F);
	cv::Mat ABt_D = cv::Mat(3, 1, CV_64F);
	cv::Mat ABt_U = cv::Mat(3, 3, CV_64F);
	cv::Mat ABt_V = cv::Mat(3, 3, CV_64F);

	//cvSetZero(&ABt);
	double* abt = (double*)ABt.data;
	for (int i = 0; i < number_of_correspondences; i++) {
		double * pc = pcs + 3 * i;
		double * pw = pws + 3 * i;

		for (int j = 0; j < 3; j++) {
			abt[3 * j    ] += (pc[j] - pc0[j]) * (pw[0] - pw0[0]);
			abt[3 * j + 1] += (pc[j] - pc0[j]) * (pw[1] - pw0[1]);
			abt[3 * j + 2] += (pc[j] - pc0[j]) * (pw[2] - pw0[2]);
		}
	}

	cv::SVD svd(ABt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
	//cvSVD(&ABt, &ABt_D, &ABt_U, &ABt_V, CV_SVD_MODIFY_A);
	ABt_D = svd.w;
	ABt_U = svd.u;
	ABt_V = svd.vt.t();

	double* abt_d = (double*)ABt_D.data;
	double* abt_u = (double*)ABt_U.data;
	double* abt_v = (double*)ABt_V.data;

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			R[i][j] = dot(abt_u + 3 * i, abt_v + 3 * j);

	const double det =
		R[0][0] * R[1][1] * R[2][2] + R[0][1] * R[1][2] * R[2][0] + R[0][2] * R[1][0] * R[2][1] -
		R[0][2] * R[1][1] * R[2][0] - R[0][1] * R[1][0] * R[2][2] - R[0][0] * R[1][2] * R[2][1];

	if (det < 0) {
		R[2][0] = -R[2][0];
		R[2][1] = -R[2][1];
		R[2][2] = -R[2][2];
	}

	t[0] = pc0[0] - dot(R[0], pw0);
	t[1] = pc0[1] - dot(R[1], pw0);
	t[2] = pc0[2] - dot(R[2], pw0);
}

void epnp::print_pose(const double R[3][3], const double t[3])
{
	cout << R[0][0] << " " << R[0][1] << " " << R[0][2] << " " << t[0] << endl;
	cout << R[1][0] << " " << R[1][1] << " " << R[1][2] << " " << t[1] << endl;
	cout << R[2][0] << " " << R[2][1] << " " << R[2][2] << " " << t[2] << endl;
}

void epnp::solve_for_sign(void)
{
	if (pcs[2] < 0.0) {
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 3; j++)
				ccs[i][j] = -ccs[i][j];

		for (int i = 0; i < number_of_correspondences; i++) {
			pcs[3 * i] = -pcs[3 * i];
			pcs[3 * i + 1] = -pcs[3 * i + 1];
			pcs[3 * i + 2] = -pcs[3 * i + 2];
		}
	}
}

double epnp::compute_R_and_t(const double * ut, const double * betas, double R[3][3], double t[3])
{
	compute_ccs(betas, ut);
	compute_pcs();

	solve_for_sign();

	estimate_R_and_t(R, t);

	return reprojection_error(R, t);
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_1 = [B11 B12     B13         B14]

void epnp::find_betas_approx_1(const cv::Mat& L_6x10, const cv::Mat& Rho, double * betas)
{
	//double l_6x4[6 * 4], b4[4];
	cv::Mat L_6x4 = cv::Mat(6, 4, CV_64F);
	cv::Mat B4 = cv::Mat(4, 1, CV_64F);

	for (int i = 0; i < 6; i++) {
		L_6x4.at<double>(i, 0) = L_6x10.at<double>(i, 0);
		L_6x4.at<double>(i, 1) = L_6x10.at<double>(i, 1);
		L_6x4.at<double>(i, 2) = L_6x10.at<double>(i, 3);
		L_6x4.at<double>(i, 3) = L_6x10.at<double>(i, 6);
		/*cvmSet(&L_6x4, i, 0, cvmGet(L_6x10, i, 0));
		cvmSet(&L_6x4, i, 1, cvmGet(L_6x10, i, 1));
		cvmSet(&L_6x4, i, 2, cvmGet(L_6x10, i, 3));
		cvmSet(&L_6x4, i, 3, cvmGet(L_6x10, i, 6));*/
	}

	//cvSolve(&L_6x4, Rho, &B4, CV_SVD);
	cv::solve(L_6x4, Rho, B4, cv::DECOMP_SVD);

	double* l_6x4 = (double*)L_6x4.data;
	double* b4 = (double*)B4.data;
	if (b4[0] < 0) {
		betas[0] = sqrt(-b4[0]);
		betas[1] = -b4[1] / betas[0];
		betas[2] = -b4[2] / betas[0];
		betas[3] = -b4[3] / betas[0];
	}
	else {
		betas[0] = sqrt(b4[0]);
		betas[1] = b4[1] / betas[0];
		betas[2] = b4[2] / betas[0];
		betas[3] = b4[3] / betas[0];
	}
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_2 = [B11 B12 B22                            ]

void epnp::find_betas_approx_2(const cv::Mat& L_6x10, const cv::Mat& Rho, double * betas)
{
	/*double l_6x3[6 * 3], b3[3];
	cv::Mat L_6x3 = cv::Mat(6, 3, CV_64F, l_6x3);
	cv::Mat B3 = cv::Mat(3, 1, CV_64F, b3);*/
	
	cv::Mat L_6x3 = cv::Mat(6, 3, CV_64F);
	cv::Mat B3 = cv::Mat(3, 1, CV_64F);

	for (int i = 0; i < 6; i++) {
		L_6x3.at<double>(i, 0) = L_6x10.at<double>(i, 0);
		L_6x3.at<double>(i, 1) = L_6x10.at<double>(i, 1);
		L_6x3.at<double>(i, 2) = L_6x10.at<double>(i, 2);
		/*cvmSet(&L_6x3, i, 0, cvmGet(L_6x10, i, 0));
		cvmSet(&L_6x3, i, 1, cvmGet(L_6x10, i, 1));
		cvmSet(&L_6x3, i, 2, cvmGet(L_6x10, i, 2));*/
	}

	//cvSolve(&L_6x3, Rho, &B3, CV_SVD);
	cv::solve(L_6x3, Rho, B3, cv::DECOMP_SVD);

	double* l_6x3 = (double*)L_6x3.data;
	double* b3 = (double*)B3.data;

	if (b3[0] < 0) {
		betas[0] = sqrt(-b3[0]);
		betas[1] = (b3[2] < 0) ? sqrt(-b3[2]) : 0.0;
	}
	else {
		betas[0] = sqrt(b3[0]);
		betas[1] = (b3[2] > 0) ? sqrt(b3[2]) : 0.0;
	}

	if (b3[1] < 0) betas[0] = -betas[0];

	betas[2] = 0.0;
	betas[3] = 0.0;
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_3 = [B11 B12 B22 B13 B23                    ]

void epnp::find_betas_approx_3(const cv::Mat& L_6x10, const cv::Mat& Rho, double * betas)
{
	/*cv::Mat L_6x5 = cv::Mat(6, 5, CV_64F, l_6x5);
	cv::Mat B5 = cv::Mat(5, 1, CV_64F, b5);*/
	cv::Mat L_6x5 = cv::Mat(6, 5, CV_64F);
	cv::Mat B5 = cv::Mat(5, 1, CV_64F);
	
	for (int i = 0; i < 6; i++) {
		L_6x5.at<double>(i, 0) = L_6x10.at<double>(i, 0);
		L_6x5.at<double>(i, 1) = L_6x10.at<double>(i, 1);
		L_6x5.at<double>(i, 2) = L_6x10.at<double>(i, 2);
		L_6x5.at<double>(i, 3) = L_6x10.at<double>(i, 3);
		L_6x5.at<double>(i, 4) = L_6x10.at<double>(i, 4);
		/*cvmSet(&L_6x5, i, 0, cvmGet(L_6x10, i, 0));
		cvmSet(&L_6x5, i, 1, cvmGet(L_6x10, i, 1));
		cvmSet(&L_6x5, i, 2, cvmGet(L_6x10, i, 2));
		cvmSet(&L_6x5, i, 3, cvmGet(L_6x10, i, 3));
		cvmSet(&L_6x5, i, 4, cvmGet(L_6x10, i, 4));*/
	}

	//cvSolve(&L_6x5, Rho, &B5, CV_SVD);
	cv::solve(L_6x5, Rho, B5, cv::DECOMP_SVD);

	double* l_6x5 = (double*)L_6x5.data;
	double* b5 = (double*)B5.data;

	if (b5[0] < 0) {
		betas[0] = sqrt(-b5[0]);
		betas[1] = (b5[2] < 0) ? sqrt(-b5[2]) : 0.0;
	}
	else {
		betas[0] = sqrt(b5[0]);
		betas[1] = (b5[2] > 0) ? sqrt(b5[2]) : 0.0;
	}
	if (b5[1] < 0) betas[0] = -betas[0];
	betas[2] = b5[3] / betas[0];
	betas[3] = 0.0;
}

void epnp::compute_L_6x10(const double * ut, double * l_6x10)
{
	const double * v[4];

	v[0] = ut + 12 * 11;
	v[1] = ut + 12 * 10;
	v[2] = ut + 12 * 9;
	v[3] = ut + 12 * 8;

	double dv[4][6][3];

	for (int i = 0; i < 4; i++) {
		int a = 0, b = 1;
		for (int j = 0; j < 6; j++) {
			dv[i][j][0] = v[i][3 * a] - v[i][3 * b];
			dv[i][j][1] = v[i][3 * a + 1] - v[i][3 * b + 1];
			dv[i][j][2] = v[i][3 * a + 2] - v[i][3 * b + 2];

			b++;
			if (b > 3) {
				a++;
				b = a + 1;
			}
		}
	}

	for (int i = 0; i < 6; i++) {
		double * row = l_6x10 + 10 * i;

		row[0] = dot(dv[0][i], dv[0][i]);
		row[1] = 2.0f * dot(dv[0][i], dv[1][i]);
		row[2] = dot(dv[1][i], dv[1][i]);
		row[3] = 2.0f * dot(dv[0][i], dv[2][i]);
		row[4] = 2.0f * dot(dv[1][i], dv[2][i]);
		row[5] = dot(dv[2][i], dv[2][i]);
		row[6] = 2.0f * dot(dv[0][i], dv[3][i]);
		row[7] = 2.0f * dot(dv[1][i], dv[3][i]);
		row[8] = 2.0f * dot(dv[2][i], dv[3][i]);
		row[9] = dot(dv[3][i], dv[3][i]);
	}
}

void epnp::compute_rho(double * rho)
{
	rho[0] = dist2(cws[0], cws[1]);
	rho[1] = dist2(cws[0], cws[2]);
	rho[2] = dist2(cws[0], cws[3]);
	rho[3] = dist2(cws[1], cws[2]);
	rho[4] = dist2(cws[1], cws[3]);
	rho[5] = dist2(cws[2], cws[3]);
}

void epnp::compute_A_and_b_gauss_newton(const double * l_6x10, const double * rho, double betas[4], cv::Mat& A, cv::Mat& b)
{
	for (int i = 0; i < 6; i++) {
		const double * rowL = l_6x10 + i * 10;
		//double * rowA = A->data.db + i * 4;
		double * rowA = (double *)A.data + i * 4;

		rowA[0] = 2 * rowL[0] * betas[0] + rowL[1] * betas[1] + rowL[3] * betas[2] + rowL[6] * betas[3];
		rowA[1] = rowL[1] * betas[0] + 2 * rowL[2] * betas[1] + rowL[4] * betas[2] + rowL[7] * betas[3];
		rowA[2] = rowL[3] * betas[0] + rowL[4] * betas[1] + 2 * rowL[5] * betas[2] + rowL[8] * betas[3];
		rowA[3] = rowL[6] * betas[0] + rowL[7] * betas[1] + rowL[8] * betas[2] + 2 * rowL[9] * betas[3];

		b.at<double>(i, 0) = rho[i] -
			(
				rowL[0] * betas[0] * betas[0] +
				rowL[1] * betas[0] * betas[1] +
				rowL[2] * betas[1] * betas[1] +
				rowL[3] * betas[0] * betas[2] +
				rowL[4] * betas[1] * betas[2] +
				rowL[5] * betas[2] * betas[2] +
				rowL[6] * betas[0] * betas[3] +
				rowL[7] * betas[1] * betas[3] +
				rowL[8] * betas[2] * betas[3] +
				rowL[9] * betas[3] * betas[3]
				);
		/*cvmSet(b, i, 0, rho[i] -
			(
				rowL[0] * betas[0] * betas[0] +
				rowL[1] * betas[0] * betas[1] +
				rowL[2] * betas[1] * betas[1] +
				rowL[3] * betas[0] * betas[2] +
				rowL[4] * betas[1] * betas[2] +
				rowL[5] * betas[2] * betas[2] +
				rowL[6] * betas[0] * betas[3] +
				rowL[7] * betas[1] * betas[3] +
				rowL[8] * betas[2] * betas[3] +
				rowL[9] * betas[3] * betas[3]
				));*/
	}
}

void epnp::gauss_newton(const cv::Mat& L_6x10, const cv::Mat& Rho, double betas[4])
{
	const int iterations_number = 5;

	/*double a[6 * 4], b[6], x[4];
	cv::Mat A = cv::Mat(6, 4, CV_64F, a);
	cv::Mat B = cv::Mat(6, 1, CV_64F, b);
	cv::Mat X = cv::Mat(4, 1, CV_64F, x);*/

	cv::Mat A = cv::Mat(6, 4, CV_64F);
	cv::Mat B = cv::Mat(6, 1, CV_64F);
	cv::Mat X = cv::Mat(4, 1, CV_64F);

	double* a = (double*)A.data;
	double* b = (double*)B.data;
	double* x = (double*)X.data;
	for (int k = 0; k < iterations_number; k++) {
		//compute_A_and_b_gauss_newton(L_6x10->data.db, Rho->data.db, betas, &A, &B);
		compute_A_and_b_gauss_newton((double*)L_6x10.data, (double*)Rho.data, betas, A, B);
		qr_solve(A, B, X);
		//qr_solve(&A, &B, &X);

		for (int i = 0; i < 4; i++)
			betas[i] += x[i];
	}
}

void epnp::qr_solve(cv::Mat& A, cv::Mat& b, cv::Mat& X)
{
	static int max_nr = 0;
	static double * A1, *A2;

	/*const int nr = A->rows;
	const int nc = A->cols;*/
	const int nr = A.rows;
	const int nc = A.cols;

	if (max_nr != 0 && max_nr < nr) {
		delete[] A1;
		delete[] A2;
	}
	if (max_nr < nr) {
		max_nr = nr;
		A1 = new double[nr];
		A2 = new double[nr];
	}

	//double * pA = A->data.db, *ppAkk = pA;
	double * pA = (double*)A.data, *ppAkk = pA;
	for (int k = 0; k < nc; k++) {
		double * ppAik = ppAkk, eta = fabs(*ppAik);
		for (int i = k + 1; i < nr; i++) {
			double elt = fabs(*ppAik);
			if (eta < elt) eta = elt;
			ppAik += nc;
		}

		if (eta == 0) {
			A1[k] = A2[k] = 0.0;
			cerr << "God damnit, A is singular, this shouldn't happen." << endl;
			return;
		}
		else {
			double * ppAik = ppAkk, sum = 0.0, inv_eta = 1. / eta;
			for (int i = k; i < nr; i++) {
				*ppAik *= inv_eta;
				sum += *ppAik * *ppAik;
				ppAik += nc;
			}
			double sigma = sqrt(sum);
			if (*ppAkk < 0)
				sigma = -sigma;
			*ppAkk += sigma;
			A1[k] = sigma * *ppAkk;
			A2[k] = -eta * sigma;
			for (int j = k + 1; j < nc; j++) {
				double * ppAik = ppAkk, sum = 0;
				for (int i = k; i < nr; i++) {
					sum += *ppAik * ppAik[j - k];
					ppAik += nc;
				}
				double tau = sum / A1[k];
				ppAik = ppAkk;
				for (int i = k; i < nr; i++) {
					ppAik[j - k] -= tau * *ppAik;
					ppAik += nc;
				}
			}
		}
		ppAkk += nc + 1;
	}

	// b <- Qt b
	//double * ppAjj = pA, *pb = b->data.db;
	double * ppAjj = pA, *pb = (double*)b.data;
	for (int j = 0; j < nc; j++) {
		double * ppAij = ppAjj, tau = 0;
		for (int i = j; i < nr; i++) {
			tau += *ppAij * pb[i];
			ppAij += nc;
		}
		tau /= A1[j];
		ppAij = ppAjj;
		for (int i = j; i < nr; i++) {
			pb[i] -= tau * *ppAij;
			ppAij += nc;
		}
		ppAjj += nc + 1;
	}

	// X = R-1 b
	//double * pX = X->data.db;
	double * pX = (double*)X.data;
	pX[nc - 1] = pb[nc - 1] / A2[nc - 1];
	for (int i = nc - 2; i >= 0; i--) {
		double * ppAij = pA + i * nc + (i + 1), sum = 0;

		for (int j = i + 1; j < nc; j++) {
			sum += *ppAij * pX[j];
			ppAij++;
		}
		pX[i] = (pb[i] - sum) / A2[i];
	}
}



void epnp::relative_error(double & rot_err, double & transl_err,
	const double Rtrue[3][3], const double ttrue[3],
	const double Rest[3][3], const double test[3])
{
	double qtrue[4], qest[4];

	mat_to_quat(Rtrue, qtrue);
	mat_to_quat(Rest, qest);

	double rot_err1 = sqrt((qtrue[0] - qest[0]) * (qtrue[0] - qest[0]) +
		(qtrue[1] - qest[1]) * (qtrue[1] - qest[1]) +
		(qtrue[2] - qest[2]) * (qtrue[2] - qest[2]) +
		(qtrue[3] - qest[3]) * (qtrue[3] - qest[3])) /
		sqrt(qtrue[0] * qtrue[0] + qtrue[1] * qtrue[1] + qtrue[2] * qtrue[2] + qtrue[3] * qtrue[3]);

	double rot_err2 = sqrt((qtrue[0] + qest[0]) * (qtrue[0] + qest[0]) +
		(qtrue[1] + qest[1]) * (qtrue[1] + qest[1]) +
		(qtrue[2] + qest[2]) * (qtrue[2] + qest[2]) +
		(qtrue[3] + qest[3]) * (qtrue[3] + qest[3])) /
		sqrt(qtrue[0] * qtrue[0] + qtrue[1] * qtrue[1] + qtrue[2] * qtrue[2] + qtrue[3] * qtrue[3]);

	rot_err = min(rot_err1, rot_err2);

	transl_err =
		sqrt((ttrue[0] - test[0]) * (ttrue[0] - test[0]) +
		(ttrue[1] - test[1]) * (ttrue[1] - test[1]) +
			(ttrue[2] - test[2]) * (ttrue[2] - test[2])) /
		sqrt(ttrue[0] * ttrue[0] + ttrue[1] * ttrue[1] + ttrue[2] * ttrue[2]);
}

void epnp::mat_to_quat(const double R[3][3], double q[4])
{
	double tr = R[0][0] + R[1][1] + R[2][2];
	double n4;

	if (tr > 0.0f) {
		q[0] = R[1][2] - R[2][1];
		q[1] = R[2][0] - R[0][2];
		q[2] = R[0][1] - R[1][0];
		q[3] = tr + 1.0f;
		n4 = q[3];
	}
	else if ((R[0][0] > R[1][1]) && (R[0][0] > R[2][2])) {
		q[0] = 1.0f + R[0][0] - R[1][1] - R[2][2];
		q[1] = R[1][0] + R[0][1];
		q[2] = R[2][0] + R[0][2];
		q[3] = R[1][2] - R[2][1];
		n4 = q[0];
	}
	else if (R[1][1] > R[2][2]) {
		q[0] = R[1][0] + R[0][1];
		q[1] = 1.0f + R[1][1] - R[0][0] - R[2][2];
		q[2] = R[2][1] + R[1][2];
		q[3] = R[2][0] - R[0][2];
		n4 = q[1];
	}
	else {
		q[0] = R[2][0] + R[0][2];
		q[1] = R[2][1] + R[1][2];
		q[2] = 1.0f + R[2][2] - R[0][0] - R[1][1];
		q[3] = R[0][1] - R[1][0];
		n4 = q[2];
	}
	double scale = 0.5f / double(sqrt(n4));

	q[0] *= scale;
	q[1] *= scale;
	q[2] *= scale;
	q[3] *= scale;
}