#include "rectify.h"

void intrinsicExtrinsic::printError(string errorSource, string errorMsg)
{
    cerr << "[ " <<  errorSource << " ] caused a fault: " << errorMsg << endl;
}

bool intrinsicExtrinsic::readRectifyParams(string confPath, string &xmlImagesl, string &xmlImagesr, string &savePath,
                                          string &ymlIntrinsic1, string &ymlIntrinsic2, string &ymlExtrinsics)
{
    bool readSuccess = false;
    libconfig::Config cfg;
    stringstream errorMsg;

    try
    {
        cfg.readFile(confPath.c_str());

        xmlImagesl = (const char *)cfg.lookup("imagesXmlLeft");
        xmlImagesr = (const char *)cfg.lookup("imagesXmlRight");
        savePath = (const char *)cfg.lookup("savePath");
        ymlIntrinsic1 = (const char *)cfg.lookup("ymlIntrinsic1");
        ymlIntrinsic2 = (const char *)cfg.lookup("ymlIntrinsic2");
        ymlExtrinsics = (const char *)cfg.lookup("ymlExtrinsics");

        readSuccess = true;
    }
    catch(const libconfig::FileIOException &fioex)
    {
        printError("readStereoParams", "I/O error while reading file.");
    }
    catch(const libconfig::ParseException &pex)
    {
        errorMsg << "Parse error at " << pex.getFile() << ":" << pex.getLine() << " - " << pex.getError();
        printError("readStereoParams", errorMsg.str());
    }
    catch(const libconfig::SettingNotFoundException &nfex)
    {
        printError("readStereoParams", "Error in the configuration file! Missing command or incorrect spelling!"
                   "Necessary parameters: 'imagesXml' (string), 'chessboardWidth' (int), 'chessboardHeight' (int),"
                   " 'subPixSize' (int), 'savePath' (string), 'intrinsic1Yml (string)', 'intrinsic2Yml (string)',"
                   " 'ymlExtrinsics (string)'");
    }

    return readSuccess;
}

bool intrinsicExtrinsic::readMonoParams(string confPath, Size &boardSize, string &xmlImages,
                                          Size &subPixSize, string &savePath)
{
    bool readSuccess = false;
    libconfig::Config cfg;
    stringstream errorMsg;
    int width, height, scale;

    try
    {
        cfg.readFile(confPath.c_str());

        xmlImages = (const char *)cfg.lookup("imagesXml");
        savePath = (const char *)cfg.lookup("savePath");
        width = (int) cfg.lookup("chessboardWidth");
        height = (int) cfg.lookup("chessboardHeight");
        scale = (int) cfg.lookup("subPixSize");

        subPixSize = Size(scale, scale);
        boardSize = Size(width, height);

        readSuccess = true;
    }
    catch(const libconfig::FileIOException &fioex)
    {
        printError("readStereoParams", "I/O error while reading file.");
    }
    catch(const libconfig::ParseException &pex)
    {
        errorMsg << "Parse error at " << pex.getFile() << ":" << pex.getLine() << " - " << pex.getError();
        printError("readStereoParams", errorMsg.str());
    }
    catch(const libconfig::SettingNotFoundException &nfex)
    {
        printError("readStereoParams", "Error in the configuration file! Missing command or incorrect spelling!"
                   "Necessary parameters: 'imagesXml' (string), 'chessboardWidth' (int), 'chessboardHeight' (int),"
                   " 'subPixSize' (int), 'savePath' (string), 'intrinsic1Yml (string)', 'intrinsic2Yml (string)'");
    }

    return readSuccess;
}

bool intrinsicExtrinsic::loadStereoExtrinsics(string ymlExtrinsics, Mat &R, Mat &T)
{
    FileStorage fStorage(ymlExtrinsics.c_str(), FileStorage::READ);
    bool readParams = false;
//	Rect roi1, roi2;

    if (fStorage.isOpened())
    {
        fStorage["R"] >> R;
        fStorage["T"] >> T;

        readParams = true;

        fStorage.release();
    }

//    if (fStorage.isOpened())
//    {
//        fStorage["R1"] >> R1;
//        fStorage["P1"] >> P1;
//        fStorage["R2"] >> R2;
//        fStorage["P2"] >> P2;

//        fStorage["roi1"] >> roi1;
//        fStorage["roi2"] >> roi2;

//		Point2i rectCorner1(max(roi1.x, roi2.x), max(roi1.y, roi2.y));

//        Point2i rectCorner2(min(roi1.x + roi1.width, roi2.x + roi2.width),
//                            min(roi1.y + roi1.height, roi2.y + roi2.height));

//        validRoi = Rect(rectCorner1.x, rectCorner1.y,
//                 rectCorner2.x - rectCorner1.x, rectCorner2.y - rectCorner1.y);

//        readParams = true;

//        fStorage.release();
//    }

    return readParams;
}

bool intrinsicExtrinsic::loadStereoIntrinsics(string &ymlIntrinsic1, string &ymlIntrinsic2,
                                              Mat &cameraMatrix1, Mat &cameraMatrix2,
                                              Mat &distCoeffs1,  Mat &distCoeffs2)
{
    bool openned;

    FileStorage fStorage1(ymlIntrinsic1.c_str(), FileStorage::READ);
    FileStorage fStorage2(ymlIntrinsic2.c_str(), FileStorage::READ);

    openned = fStorage1.isOpened() && fStorage2.isOpened();

    if (openned)
    {
        fStorage1["Camera Matrix"] >> cameraMatrix1;
        fStorage1["Distortion Coefficients"] >> distCoeffs1;
        fStorage1.release();

        fStorage2["Camera Matrix"] >> cameraMatrix2;
        fStorage2["Distortion Coefficients"] >> distCoeffs2;
        fStorage2.release();
    }

    return openned;
}

bool intrinsicExtrinsic::loadImageList(string file, vector<string> &list)
{
    bool loadSuccess = false;
    FileNode fNode;
    FileStorage fStorage(file, FileStorage::READ);

    if (fStorage.isOpened())
    {
        fNode = fStorage.getFirstTopLevelNode();

        if (fNode.type() == FileNode::SEQ)
        {
            for(FileNodeIterator iterator = fNode.begin(); iterator != fNode.end(); ++iterator)
            {
                list.push_back((string) *iterator);
            }

            loadSuccess = true;
        }
    }

    return loadSuccess;
}

bool intrinsicExtrinsic::loadImages(vector<string> fileList, vector<Mat> &images)
{
    bool emptyImage = false;
    for (int i = 0; i < fileList.size() && !emptyImage; ++i)
    {
        Mat curImg = imread(fileList[i]);
        if (!curImg.empty())
        {
            images.push_back(curImg);
        }
        else
        {
            emptyImage = true;
        }
    }

    return (!emptyImage && images.size() > 0);
}

bool intrinsicExtrinsic::undistortStereoImages(vector<Mat> &inputImagesLeft, vector<Mat> &inputImagesRight,
                                               vector<Mat> &outputImagesLeft, vector<Mat> &outputImagesRight,
                                               Mat &cameraMatrix1, Mat &cameraMatrix2,
                                               Mat &distCoeffs1,  Mat &distCoeffs2)
{
    bool undistortSuccess = false;

    for(int i =0; i < (inputImagesLeft.size()/2) ;i++)
    {
        Mat undistort1, undistort2;
        undistort(inputImagesLeft[i], undistort1, cameraMatrix1, distCoeffs1);
        undistort(inputImagesRight[i], undistort2, cameraMatrix2, distCoeffs2);

        outputImagesLeft.push_back(undistort1);
        outputImagesRight.push_back(undistort2);
    }

    undistortSuccess = true;

    return undistortSuccess;

}
