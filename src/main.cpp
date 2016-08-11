#include "rectify.h"
#include <boost/filesystem.hpp>

using namespace intrinsicExtrinsic;

#include <string>
#include <iomanip>

int main(int argc, char** argv)
{
    bool ok = false;
    string xmlImagesL,
           xmlImagesR,
           savePath,
           ymlIntrinsic1,
           ymlIntrinsic2,
           ymlExtrinsics;
    vector<string> fileListLeft, fileListRight;
    vector<Mat> imagesLeft, imagesRight;
    vector<Mat> undistortedImagesLeft, undistortedImagesRight;
    Mat cameraMatrix1, cameraMatrix2, distCoeffs1, distCoeffs2;
    Mat noDist = Mat::zeros(5,1, CV_32F);
    Size imageSize;
    Mat R, T;
//    Mat R1, R2, P1, P2;
    Rect validRoi;

    if (argc >= 2)
    {
        cout << "[ImageRectify] read params!" << endl;
        ok = readRectifyParams(string(argv[1]), xmlImagesL, xmlImagesR, savePath, ymlIntrinsic1, ymlIntrinsic2, ymlExtrinsics);

        if (!ok)
            cout << "[ImageRectify] error! could not read params!" << endl;
    }

    if (ok)
    {
        cout << "[ImageRectify] load image list!" << endl;
        bool okleft = loadImageList(xmlImagesL, fileListLeft);
        bool okright = loadImageList(xmlImagesR, fileListRight);
        ok = okleft & okright;

        if (!ok)
            cout << "[ImageRectify] error! could not load image list!" << endl;
    }

    if (ok)
    {
        cout << "[ImageRectify] load images!" << endl;
        bool okleft = loadImages(fileListLeft, imagesLeft);
        bool okright = loadImages(fileListRight, imagesRight);
        ok = okleft & okright;

        if (!ok)
            cout << "[ImageRectify] error! could not load images!" << endl;
    }

    if (ok)
    {
        imageSize.height = ((Mat)imagesLeft.at(0)).rows;
        imageSize.width = ((Mat)imagesLeft.at(0)).cols;
    }

    if (ok)
    {
        cout << "[ImageRectify] load intrinsics!" << endl;
        ok = loadStereoIntrinsics(ymlIntrinsic1, ymlIntrinsic2,
                             cameraMatrix1, cameraMatrix2, distCoeffs1, distCoeffs2);

        if (!ok)
            cout << "[ImageRectify] error! could not load intrinsics!" << endl;
    }

    if (ok)
    {
        cout << "[ImageRectify] load extrinsics!" << endl;
//        ok = loadStereoExtrinsics(ymlExtrinsics, R1, P1, R2, P2, validRoi);
        ok = loadStereoExtrinsics(ymlExtrinsics, R, T);

        if (!ok)
            cout << "[ImageRectify] error! could not load extrinsics!" << endl;
    }

    if (ok)
    {
        cout << "[ImageRectify] undistort images!" << endl;
        ok = undistortStereoImages(imagesLeft, imagesRight,
                                   undistortedImagesLeft, undistortedImagesRight,
                                   cameraMatrix1, cameraMatrix2, distCoeffs1, distCoeffs2);

        if (!ok)
            cout << "[ImageRectify] error! could not undistort images!" << endl;
    }

    // Remap and save
    if (ok)
    {
        Mat R1, R2, P1, P2, Q;
        Rect roi1, roi2;

        stereoRectify(cameraMatrix1, distCoeffs1,
                      cameraMatrix2, distCoeffs2,
                      imageSize, R, T,
                      R1, R2, P1, P2, Q,
                      CALIB_ZERO_DISPARITY, 1, imageSize, &roi1, &roi2);

        Mat rmap[2][2];
        initUndistortRectifyMap(cameraMatrix1, noDist, R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
        initUndistortRectifyMap(cameraMatrix2, noDist, R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
        int imgCount = undistortedImagesLeft.size();

		boost::filesystem::path dir(savePath);
        boost::filesystem::create_directories(dir);

        Point2i rectCorner1(max(roi1.x, roi2.x), max(roi1.y, roi2.y));
        Point2i rectCorner2(min(roi1.x + roi1.width, roi2.x + roi2.width),
                            min(roi1.y + roi1.height, roi2.y + roi2.height));
        Rect validRoi = Rect(rectCorner1.x, rectCorner1.y,
                             rectCorner2.x - rectCorner1.x, rectCorner2.y - rectCorner1.y);

        for(int i = 0; i < imgCount; i++)
        {            
            cout << "[ImageRectify] rectify image " << (i + 1) << " of " << imgCount << endl;

            Mat imgLeft = undistortedImagesLeft[i], remapImgLeft, rectImgLeft,
                imgRight = undistortedImagesRight[i], remapImgRight, rectImgRight;
            remap(imgLeft, remapImgLeft, rmap[0][0], rmap[0][1], CV_INTER_LINEAR);
            remap(imgRight, remapImgRight, rmap[1][0], rmap[1][1], CV_INTER_LINEAR);

            stringstream filenameLeft, filenameRight;

            filenameLeft << savePath << "/image_0/" << std::setw(6) << std::setfill('0') << i << ".png";
            filenameRight << savePath << "/image_1/" << std::setw(6) << std::setfill('0') << i << ".png";

            rectImgLeft = remapImgLeft(validRoi);
            rectImgRight = remapImgRight(validRoi);
            imwrite(filenameLeft.str(), rectImgLeft);
            imwrite(filenameRight.str(), rectImgRight);
        }
    }

    return 0;
}
