//
//  TemplateMatching.cpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/03/22.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#include "TemplateMatcher.hpp"

Mat TemplateMatcher::pyramid(Mat src, int n)  //（入力画像，出力画像の大きさ[1/(2^n)]）
{
    Mat dst = src;
    for (int i = 0; i < n; i++)	pyrDown(dst, dst);  // pyrDownは元画像の半分の画像を生成する標準関数
    return dst;
}

void TemplateMatcher::setTemplate(Mat temp){
    tmp = temp.clone();
    width = tmp.size().width;
    height = tmp.size().height;
    tmp1_2 = pyramid(tmp, 1);  // ２分の１のテンプレート画像
    tmp1_4 = pyramid(tmp, 2);  // ４分の１のテンプレート画像
}

void TemplateMatcher::coaseToFine(Mat src, Mat tmp, double &value, Point &point ,int n){
    Mat result;
    Rect rect;
    rect.x = point.x - width / n;
    rect.y = point.y - height / n;
    rect.width = width / n * 2;
    rect.height = height / n * 2;
    
    if(isArea(src, rect)){
        Mat sr(src, rect);
        matchTemplate(sr, tmp, result, TM_CCORR_NORMED);
        minMaxLoc(result, NULL, &value, NULL, &point, Mat());
        point.x += rect.x;
        point.y += rect.y;
    }
}

void TemplateMatcher::TM(Mat src)
{
    evaluate = 0.0f;
    estimate.x = 0;
    estimate.y = 0;
    
    Point point;
    double value;
    Mat result;
    
    matchTemplate(pyramid(src, 2), tmp1_4, result, TM_CCORR_NORMED);
    minMaxLoc(result, NULL, &value, NULL, &point, Mat());
    
    point.x *= 2;
    point.y *= 2;
    
    Mat src1_2 = pyramid(src, 1);
    coaseToFine(src1_2, tmp1_2, value, point, 2);
    
    point.x *= 2;
    point.y *= 2;
    
    coaseToFine(src, tmp, value, point, 1);
    
    evaluate = static_cast<float>(value);
    estimate.x = point.x;
    estimate.y = point.y;
}

void TemplateMatcher::drawResult(Mat &rsc){
    // テンプレートマッチング位置を四角で囲む
    rectangle(rsc, Point(estimate.x, estimate.y), Point(estimate.x + tmp.cols, estimate.y + tmp.rows), Scalar(0, 0, 200, 255), 1, CV_AA);
}


