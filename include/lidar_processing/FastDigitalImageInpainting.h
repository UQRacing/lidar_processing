/*
 * Source: https://github.com/Mugichoko445/Fast-Digital-Image-Inpainting
 * MIT licence
 * Small modifications made 2022 by Matt Young for UQRacing
 */
#pragma once

#include <opencv2/opencv.hpp>

namespace uqr {
    void fastInpaint(const cv::Mat &src, const cv::Mat &mask, cv::Mat &dst, int maxNumOfIter = 100) {
        assert(src.type() == mask.type() && mask.type() == CV_8UC3);
        assert(src.size() == mask.size());

        static const float a(0.073235f);
        static const float b(0.176765f);
        static const cv::Mat K = (cv::Mat_<float>(3, 3) << a, b, a, b, 0.0f, b, a, b, a);

        // fill in the missing region with the input's average color
        auto avgColor = cv::sum(src) / (src.cols * src.rows);
        cv::Mat avgColorMat(1, 1, CV_8UC3);
        avgColorMat.at<cv::Vec3b>(0, 0) = cv::Vec3b(avgColor[0], avgColor[1], avgColor[2]);
        cv::resize(avgColorMat, avgColorMat, src.size(), 0.0, 0.0, cv::INTER_NEAREST);
        cv::Mat result = (mask / 255).mul(src) + (1 - mask / 255).mul(avgColorMat);

        // convolution
        int bSize = K.cols / 2;
        cv::Mat kernel3ch, inWithBorder;
        result.convertTo(result, CV_32FC3);
        cv::cvtColor(K, kernel3ch, cv::COLOR_GRAY2BGR);

        cv::copyMakeBorder(result, inWithBorder, bSize, bSize, bSize, bSize, cv::BORDER_REPLICATE);
        cv::Mat resInWithBorder = cv::Mat(inWithBorder,
                                          cv::Rect(bSize, bSize, result.cols, result.rows));

        const int ch = result.channels();
        for (int itr = 0; itr < maxNumOfIter; ++itr) {
            cv::copyMakeBorder(result, inWithBorder, bSize, bSize, bSize, bSize,
                               cv::BORDER_REPLICATE);
#pragma omp parallel for default(none) shared(result) firstprivate(mask, K, ch, inWithBorder, kernel3ch)
            for (int r = 0; r < result.rows; ++r) {
                const uchar *pMask = mask.ptr(r);
                auto *pRes = result.ptr<float>(r);
                for (int c = 0; c < result.cols; ++c) {
                    if (pMask[ch * c] == 0) {
                        cv::Rect rectRoi(c, r, K.cols, K.rows);
                        cv::Mat roi(inWithBorder, rectRoi);

                        auto sum = cv::sum(kernel3ch.mul(roi));
                        pRes[ch * c + 0] = static_cast<float>(sum[0]);
                        pRes[ch * c + 1] = static_cast<float>(sum[1]);
                        pRes[ch * c + 2] = static_cast<float>(sum[2]);
                    }
                }
            }
        }

        result.convertTo(dst, CV_8UC3);
    }
}