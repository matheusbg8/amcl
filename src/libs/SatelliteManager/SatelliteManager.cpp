#include "SatelliteManager.h"

#include "../ConfigLoader/ConfigLoader.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;

/**
 * @brief RoboMap::getSonarPolyOnImg - Create the poly that cover
 * sonar field of view on satellite image.
 * @todo - This function can be improved by
 * avoid constructions of the sonar poly all the time. The poly should be
 * created once and it should be adjusted using rotation matrix.
 * @param p - Sonar position.
 * @param heading - Sonar heading in degrees where 0 degrees mean faced to north.
 * @param poly - It is an array output of the poly on image (In pixels).
 * @param nPoint - Number of points to build the poly.
 * @param range - Sonar range in meters.
 * @param bearing - Field of View (opening) in degrees of the sonar.
 */
void SatelliteManager::getSonarPolyOnImg(const Point2d &p, double heading,
                                         Point *poly, unsigned nPoint,
                                         double range, double bearing) const
{
    if(nPoint < 3) return;

    double rad = heading* M_PI/180.0, // direction, yaw
          radBearing = bearing* M_PI/180.f,
          radInc = radBearing/(nPoint-2);

    unsigned pId = 0;

    // Sonar origin
    poly[pId++] = UTM2Img(p);

    for(double currentRad = rad + radBearing/2.0;
        pId < nPoint; currentRad-= radInc)
    {
        poly[pId++] = UTM2Img(p + Point2d(sin(currentRad), cos(currentRad))*range);
    }
}

SatelliteManager::SatelliteManager()
{

}

SatelliteManager::SatelliteManager(const string &mapPath)
{
    loadMap(mapPath);
}

void SatelliteManager::changeMapResolution(double resolutionFactor)
{
    if(mapImg.rows == 0)
        return ;
// 323 - 40 -> 8.07
    resize(mapImg,mapImg,Size(), resolutionFactor,resolutionFactor,INTER_NEAREST);
    imgRef[0] *=resolutionFactor;
    imgRef[1] *=resolutionFactor;
    UTM2Img_ *= resolutionFactor;
    printf("Re-adjusted image scale to %lg meters per pixels\n", 1.0/fabs(UTM2Img_.x));

}

void SatelliteManager::adjustResolution(double metersPerPixel)
{
    changeMapResolution(1.0/(metersPerPixel *  fabs(UTM2Img_.x)));
    // e * X = 1/ne
    // X = 1/(ne * e)
}

bool SatelliteManager::loadMap(const path &mapPath)
{
    ConfigLoader config(mapPath.string().c_str());

    string str;

    if(config.getString("General", "MapImgName", &str))
    {
        path imgMapFileName = mapPath.parent_path() / str;

        cout << "MapName " << imgMapFileName.string() << endl;
        mapImg = imread(imgMapFileName.string());

        if(mapImg.empty())
        {
            cout << "Error when loading map " <<
                    imgMapFileName.string() << endl;
            assert("Problem! Image map not found");
            return false;
        }
    }

    bool hasP1UTM, hasP1Img, hasP2UTM, hasP2Img;
    hasP1UTM=hasP1Img=hasP2UTM=hasP2Img=false;

    if(config.getString("P_1", "img_x_y", &str))
    {
        cout << "P_1: img_x_y " << str << endl;
        parse2D_2_Point2d(str,imgRef[0]);
        hasP1Img=true;
    }

    if(config.getString("P_1", "utm_x_y", &str))
    {
        cout << "P_1: utm_x_y " << str << endl;
        parse2D_2_Point2d(str,UTMRef[0]);
        hasP1UTM = true;
    }

    if(config.getString("P_2", "img_x_y", &str))
    {
        cout << "P_2: img_x_y " << str << endl;
        parse2D_2_Point2d(str,imgRef[1]);
        hasP2Img = true;
    }

    if(config.getString("P_2", "utm_x_y", &str))
    {
        cout << "P_2: utm_x_y " << str << endl;
        parse2D_2_Point2d(str,UTMRef[1]);
        hasP2UTM = true;
    }

    if(!hasP1Img || !hasP1UTM || !hasP2Img || !hasP2UTM)
    {
        cout << "RoboMap::loadMap - Error!" << endl
             << "Missing reference points: |P1Img |P1UTM |P2Img |P2UTM" << endl
             << "                          |"
             << hasP1Img << " |"
             << hasP1UTM << " |"
             << hasP2Img << " |"
             << hasP2UTM << endl;
        return false;
    }

    mapName = mapPath.stem().string();

    Point2d diffImg = imgRef[1] - imgRef[0],
            diffUTM = UTMRef[1] - UTMRef[0];

    // Compute scale transform from UTM to pixel
    // taking two reference points on both coordinate system

    double UTM2ImgFac;
    UTM2Img_.x = diffImg.x/diffUTM.x;
    UTM2Img_.y = diffImg.y/diffUTM.y;

    // The scale factor should be the same for both X and Y
    // So, if it is not we take the average and save
    // the difference as an estimation of the measurement error.

    UTM2ImgFac = (fabs(UTM2Img_.x) + fabs(UTM2Img_.y))/2.0;
    UTM2ImgDiffXToY = fabs(UTM2Img_.x) - fabs(UTM2Img_.y);

    // Correcting to respect equal scale on both axis X and Y
//    if(UTM2Img_.x < 0.0) UTM2Img_.x = -UTM2ImgFac;
//    else UTM2Img_.x = UTM2ImgFac;

//    if(UTM2Img_.y < 0.0) UTM2Img_.y = -UTM2ImgFac;
//    else UTM2Img_.y = UTM2ImgFac;

    // Correcting the reference points using the new scale
//    Point2d newImgDiff(diffUTM.x * UTM2Img_.x,
//                       diffUTM.y * UTM2Img_.y),

//            refImgCorrection( fabs(newImgDiff.x) - fabs(diffImg.x) ,
//                              fabs(newImgDiff.y) - fabs(diffImg.y));

//    if(imgRef[0].x > imgRef[1].x)
//    {
//        imgRef[0].x += refImgCorrection.x/2.0;
//        imgRef[1].x -= refImgCorrection.x/2.0;
//    }
//    else
//    {
//        imgRef[0].x -= refImgCorrection.x/2.0;
//        imgRef[1].x += refImgCorrection.x/2.0;
//    }

//    if(imgRef[0].y > imgRef[1].y)
//    {
//        imgRef[0].y += refImgCorrection.y/2.0;
//        imgRef[1].y -= refImgCorrection.y/2.0;
//    }
//    else
//    {
//        imgRef[0].y -= refImgCorrection.y/2.0;
//        imgRef[1].y += refImgCorrection.y/2.0;
//    }


    printf("imgRef1 %lg %lg\n", imgRef[0].x , imgRef[0].y);
    printf("imgRef2 %lg %lg\n", imgRef[1].x , imgRef[1].y);
    printf("utmRef1 %lg %lg\n", UTMRef[0].x , UTMRef[0].y);
    printf("utmRef2 %lg %lg\n", UTMRef[1].x , UTMRef[1].y);

    printf("UTM2Img %lg %lg pixel per meters\n", UTM2Img_.x,UTM2Img_.y);
    printf("Image Scale %lg meters per pixel\n", 1.0/fabs(UTM2Img_.x));
    printf("Scale Measurement Error +-%lg pixel per meters\n", fabs(UTM2ImgDiffXToY)/2.0);

    cout << "Map loaded!" << endl;
    return true;
}

Point2d SatelliteManager::UTM2Img(const Point2d &UTMp) const
{
    Point2d p(imgRef[0].x + (UTMp.x - UTMRef[0].x)*UTM2Img_.x,
              imgRef[0].y + (UTMp.y - UTMRef[0].y)*UTM2Img_.y);
    return p;
}

Point2d SatelliteManager::Img2UTM(const Point2d &ImgP) const
{
    // First bring img point to the origin (reference point 0)
    // Second apply the scale, now it is in UTM units but out of the center
    // Third translate to the UTM reference
    Point2d p(UTMRef[0].x + (ImgP.x - imgRef[0].x)/UTM2Img_.x,
              UTMRef[0].y + (ImgP.y - imgRef[0].y)/UTM2Img_.y);
    return p;
}

void SatelliteManager::UTM2Img(Point2d *UTMpts, unsigned nPts)
{
    for(unsigned i = 0 ; i < nPts ; i++)
    {
        Point2d &p = UTMpts[i];
        p.x = imgRef[0].x + (p.x - UTMRef[0].x)*UTM2Img_.x;
        p.y = imgRef[0].y + (p.y - UTMRef[0].y)*UTM2Img_.y;
    }}

void SatelliteManager::removeOffset()
{
  Point2d UTMp0 = Img2UTM(Point2d(0.0,mapImg.rows));
  UTMRef[0]-= UTMp0;
  UTMRef[1]-= UTMp0;

}

void SatelliteManager::parse2D_2_Point2d(const string &str, Point2d &p)
{
    setlocale(LC_ALL, "C");
    double x,y;
    sscanf(str.c_str(),"%lf %lf", &x, &y);
    p.x =x;
    p.y =y;
}

void SatelliteManager::minMaxUTMValues(Point2d &min, Point2d &max)
{
    Point2d imgMin,imgMax,
           utmMin, utmMax;
    minMaxImgValues(imgMin,imgMax);

    // Convert img points (pixel) to UTM points (meters)
    utmMin = Img2UTM(imgMin);
    utmMax = Img2UTM(imgMax);

    min.x = std::min(utmMin.x, utmMax.x);
    min.y = std::min(utmMin.y, utmMax.y);

    max.x = std::max(utmMin.x, utmMax.x);
    max.y = std::max(utmMin.y, utmMax.y);
}

void SatelliteManager::minMaxImgValues(Point2d &min, Point2d &max)
{
    max = Point2d(mapImg.cols, mapImg.rows);
    min = Point2d(0.0,0.0);
}

void SatelliteManager::getWidthHeight(int *maxX, int *maxY)
{
    *maxX = mapImg.cols;
    *maxY = mapImg.rows;
}

/**
 * @brief RoboMap::cropOnPoints - This method crop the satellite image
 * in order to speed up the image processing performance. Basically it
 * crop a rect that contain all UTM points plus an extra space (outSpaceInMeters)
 * that cover eventual sensor range.
 * Obs.: Once this method is called there isn't a way to came back
 * to the original size of the satellite image. The map should be
 * loaded again.
 * @param utmPoints - The UTM points.
 * @param outSpaceInMeters - Extra space of the rect that will be used to
 * crop the image.
 */
void SatelliteManager::cropOnPoints(const vector<Point2d> &utmPoints, double outSpaceInMeters)
{
    Point2d pts[utmPoints.size()];
    for(unsigned i= 0; i < utmPoints.size();i++)
        pts[i] = utmPoints[i];

    cropOnPoints(pts, utmPoints.size(), outSpaceInMeters);
}

/**
 * @brief RoboMap::cropOnPoints - This method crop the internal
 * satellite image to speed up the image processing performance.
 *  Basically it crop a rect that contain all UTM points plus an
 * extra space (outSpaceInMeters) that cover eventual sensor range.
 * Obs.: Once this method is called there isn't a way to came back
 * to the original size of the satellite image. The map must be
 * loaded again.
 * @param utmPoints - The path of the robot in UTM.
 * @param nPts - Number of points of the path.
 * @param outSpaceInMeters - Extra space of the rect that will be used to
 * crop the image.
 */
void SatelliteManager::cropOnPoints(const Point2d *utmPoints, unsigned nPts, double outSpaceInMeters)
{
    if(nPts == 0) return;
    if(mapImg.rows == 0) return;

    Point2d max, min;

    min.x = max.x = utmPoints[0].x;
    min.y = max.y = utmPoints[0].y;

    // Find the bounding box
    for(unsigned i = 1 ; i < nPts; i++)
    {
        if(utmPoints[i].x > max.x) max.x = utmPoints[i].x;
        else if(utmPoints[i].x < min.x ) min.x = utmPoints[i].x;

        if(utmPoints[i].y > max.y) max.y = utmPoints[i].y;
        else if(utmPoints[i].y < min.y ) min.y = utmPoints[i].y;
    }

    min.x -= outSpaceInMeters;
    min.y -= outSpaceInMeters;
    max.x += outSpaceInMeters;
    max.y += outSpaceInMeters;

    cout << min << " | " << max << endl;
    cout << UTM2Img_ << endl;
    cout << imgRef[0] << endl;
    cout << imgRef[1] << endl;

    Point2d maxImg = UTM2Img(max),
            minImg = UTM2Img(min);
    cout << maxImg << " | " << minImg << endl;

    // Swap X
    if(maxImg.x < minImg.x)
    {
        maxImg.x += minImg.x;
        minImg.x = maxImg.x - minImg.x;
        maxImg.x -= minImg.x;
    }

    // Swap Y
    if(maxImg.y < minImg.y)
    {
        maxImg.y += minImg.y;
        minImg.y = maxImg.y - minImg.y;
        maxImg.y -= minImg.y;
    }

    if(minImg.x < 0.0) minImg.x = 0.0;

    if(minImg.y < 0.0) minImg.y = 0.0;

    if(maxImg.x > mapImg.cols) maxImg.x = mapImg.cols;

    if(maxImg.y > mapImg.rows) maxImg.y = mapImg.rows;

    Rect crop(minImg.x, minImg.y,
              maxImg.x-minImg.x, maxImg.y-minImg.y);

    cout << "Cropping-> " << crop << endl;

    mapImg = mapImg(crop);

    imgRef[0].x -= minImg.x;
    imgRef[0].y -= minImg.y;

    imgRef[1].x -= minImg.x;
    imgRef[1].y -= minImg.y;
}

/**
 * @brief RoboMap::cropRectUTM - This method make a rect crop on a satellite image
 * giving the UTM position its center, orientation and size.
 * @param utmP - Center position of the rect in UTM.
 * @param heightInMeters - Height of the rect in meters.
 * @param withInMeters - Width of the rect in meters.
 * @param orientationInDegrees - Orientation of the rect in degrees.
 * @return
 */
Mat SatelliteManager::cropRectUTM(Point2f &utmP, float heightInMeters, float withInMeters, float orientationInDegrees)
{
    double rad = orientationInDegrees*M_PI/180.f,
           co = cos(rad), si = sin(rad),
           halfHeightMeters = heightInMeters/2.0, halfWidthMeters = withInMeters/2.0;

    // Since it has a different scale in x and y axis
    // we are computing scale on the width and height axis
    // of out rect (it depends of its orientation)

    int imgHeight = round(heightInMeters * norm(Point2f(cos(rad)*UTM2Img_.x,sin(rad)*UTM2Img_.y))),
                    //  width in meters  *   scale to convert to image
        imgWidth = round(withInMeters * norm(Point2f(cos(rad+M_PI_2)*UTM2Img_.x,sin(rad+M_PI_2)*UTM2Img_.y)));

    Point2f imgRectCenter = UTM2Img(utmP);

    Mat T = getRotationMatrix2D(-imgRectCenter,rad,1.0),
        rotatedImg= mapImg(RotatedRect(imgRectCenter,Size2f(imgWidth,imgHeight),rad).boundingRect());


//    warpAffine(rotatedImg,rotatedImg);
    /// @todo - Finish implementation of method cropRectUTM

    return Mat();
}

Mat SatelliteManager::cropRect(Point2f &imgP, float height, float width, float oriDeeg)
{
    RotatedRect rt(imgP,Size2f(width,height),oriDeeg);
    Rect brt = rt.boundingRect();
    Size2f sz = brt.size();

    // Some cropping treatments (resize bounding box rect if out of satellite image)
    if(brt.x < 0) brt.x = 0;
    if(brt.x + brt.width > mapImg.cols)
        brt.x = mapImg.cols - brt.width - 2;

    if(brt.y < 0) brt.y = 0;
    if(brt.y + brt.height > mapImg.rows)
        brt.y = mapImg.rows - brt.height - 2;

    Mat T = getRotationMatrix2D(Point2f(sz.width/2,sz.height/2),oriDeeg,1.0);

    // Translation Correction
    // Return to the original center and translate to final rect
    T.at<double>(0,2) += -sz.width/2 + width/2;
    T.at<double>(1,2) += -sz.height/2 + height/2;

    Mat rotatedImg= mapImg(brt).clone();

    warpAffine(rotatedImg,rotatedImg,T,Size(width,height),INTER_NEAREST);

    return rotatedImg;
}

Mat SatelliteManager::cropRect(Point2f &imgP, float height, float width)
{
    Rect brt(round(imgP.x),round(imgP.y),
           height,width);

    // Some cropping treatments (resize bounding box rect if out of satellite image)
    if(brt.x < 0) brt.x = 0;
    if(brt.x + brt.width > mapImg.cols)
        brt.x = mapImg.cols - brt.width - 2;

    if(brt.y < 0) brt.y = 0;
    if(brt.y + brt.height > mapImg.rows)
        brt.y = mapImg.rows - brt.height - 2;

    return mapImg(brt).clone();
}

Mat SatelliteManager::cropSonarFoV(Point2d p, double headingDegrees,
                                   double sonarRange, double FoV,
                                   bool doPositionAdjustment) const
{
    // Get the sonar field of view polygon on satellite image
    int nPts = 15;
    Point pts[nPts];
    getSonarPolyOnImg(p,headingDegrees,pts,nPts,
                      sonarRange,FoV);

    // Find the bounding box of the poly
    int maxX,maxY, minX, minY;
    maxX = minX = pts[0].x;
    maxY = minY = pts[0].y;

    for(int i = 1 ; i < nPts;i++)
    {
        if(maxX < pts[i].x) maxX = pts[i].x;
        else
        if(minX > pts[i].x) minX = pts[i].x;

        if(maxY < pts[i].y) maxY = pts[i].y;
        else
        if(minY > pts[i].y) minY = pts[i].y;
    }


    // A boding box rect with 3 extra pixels each side
    Rect rect(minX-3 , minY-3, maxX-minX+6, maxY-minY+6);

    int top,bottom, left,right;
    Rect rectThatFits = getRectThatFitsIntoImg(rect,mapImg,
                                               top,bottom,
                                               left,right);

//    cout << "Top " << top
//         << " bottom " << bottom
//         << " left " << left
//         << " right " << right << endl;

    if(rectThatFits.width==0 || rectThatFits.height ==0)
    {
        cout << "Sat image crop error! Desired crop out of the map!!" << endl;
        return Mat();
    }

    Mat sonarFoVRect; // Rect crop from the map

    if(doPositionAdjustment)
    {
        // Change rect position if it does not fit on the map image
        rect.x += left - right;
        rect.y += top - bottom;

        sonarFoVRect = mapImg(rect);
    }else // Fill the missing part with padding
    {
        Mat cropThatFits = mapImg(rectThatFits);
        copyMakeBorder(cropThatFits,sonarFoVRect,
                       top,bottom,left,right,
                       BORDER_REFLECT_101);
    }

    // Transform the sonar FoV poly in an image mask
    Mat sonarMask(rect.height, rect.width,CV_8UC1,Scalar(0));

    int npts[] = {nPts};
    const Point* ppt[1] = { pts };
    fillPoly(sonarMask,ppt,npts,1,Scalar(255,255,255),
             8,0,Point(-rect.x,-rect.y));

    // Create the image correspondent of the sonar FoV
    Mat sonFoVImg;
    sonarFoVRect.copyTo(sonFoVImg,sonarMask);

    // Compute Sonar Position on each coordinate system
    double FoVRad = FoV*M_PI/180.0;

    Point2d sonarPositionOnImg(UTM2Img(p)),
            sonarPositionOnResult( sonarPositionOnImg.x - rect.x,
                                   sonarPositionOnImg.y - rect.y),
            sonarFoVSize(2*sonarRange*cos(M_PI_2-(FoVRad/2.0)), // width
                         sonarRange // Height
                         );

    // ===== Rotating the FoV image regarding sonar heading =============
    Size finalImgSize(abs(int(sonarFoVSize.x*UTM2Img_.x)),
                      abs(int(sonarFoVSize.y*UTM2Img_.y)));

    Mat afimTransformMatrix = getRotationMatrix2D(sonarPositionOnResult, headingDegrees,1.0);

    // Translation Correction
    afimTransformMatrix.at<double>(0,2) += -sonarPositionOnResult.x+finalImgSize.width/2;
    afimTransformMatrix.at<double>(1,2) += -sonarPositionOnResult.y+finalImgSize.height;

    // Apply affine transform and warp the image
    warpAffine( sonFoVImg,
                sonFoVImg,
                afimTransformMatrix,
                finalImgSize,
                INTER_NEAREST
               );

    return sonFoVImg;
}

/**
 * @brief SatelliteManager::getRectThatFitsIntoImg
 *  Giving a rect and an image, return an adjusted rect
 * that fits into the image. top,bottom, left and right
 * are output parameters that tells how many pixels
 * were missing on each direction. This information
 * can be used for a padding strategy.
 *  If the gave rect is completely out of the image
 * a empty rect is returned and -1 on the output
 * parameters.
 * @param r
 * @param im
 * @param top
 * @param bottom
 * @param left
 * @param right
 * @return
 */
Rect SatelliteManager::getRectThatFitsIntoImg(const Rect &r, const Mat &im,
                                              int &top, int &bottom,
                                              int &left, int &right) const
{
    top = bottom = left = right = 0;
    int minX = r.x, maxX = r.x + r.width,
        minY = r.y, maxY = r.y + r.height;

    if(maxX < 0 || minX > mapImg.cols ||
       maxY < 0 || minY > mapImg.rows)
    {
        top=bottom=left=right=-1;
        return Rect();
    }
    Rect fr(r);

    if(minX < 0)
    {
        fr.width+=minX;
        fr.x =0;
        left=-minX;
    }

    if(maxX > im.cols)
    {
        fr.width = im.cols - fr.x;
        right = maxX - im.cols;
    }

    if(minY < 0)
    {
        fr.height+=fr.y;
        fr.y = 0;
        top = -minY;
    }

    if(maxY > im.rows)
    {
        fr.height = im.rows - fr.y;
        bottom = maxY - im.rows;
    }
    return fr;
}

bool SatelliteManager::getPixel(const Point2d &UTMPp, Vec3b &pixel) const
{
  Point2d pImg = UTM2Img(UTMPp);
  if(pImg.x >=0.0 && pImg.x < mapImg.cols &&
     pImg.y >=0.0 && pImg.y < mapImg.rows)
  {
    pixel = mapImg.at<Vec3b>(int(round(pImg.y)),int(round(pImg.x)));
    return true;
  }
  return false;
}

Mat SatelliteManager::getMapImg()
{
  return mapImg.clone();
}

void SatelliteManager::setMapWhite()
{
  Rect whiteRec(0,0,mapImg.cols,mapImg.rows);
  rectangle(mapImg,whiteRec,Scalar(255,255,255),-1);
}

void SatelliteManager::normalizeMapAndSave()
{
  // Find the greater resolution (pix/m)
  // so we will never decrease image size
  // so the adjust factor must be always
  // greater than one
  if(UTM2Img_.x > UTM2Img_.y)
  { // We are going to adjust Y
    double adjustFactor = fabs(UTM2Img_.x/ UTM2Img_.y);
    double newScaleX = UTM2Img_.x,
           newScaleY = UTM2Img_.y * adjustFactor;
    cout << "New img. scale "
         << newScaleX << " , " << newScaleY << endl
         << "Adjust factor " << adjustFactor << endl;
    Mat normMap;
    resize(mapImg,normMap,Size(),1.0,adjustFactor);
    imwrite(format("NormMapRes_%lf.png",newScaleX),
            normMap);
  }else
  { // We are going to adjust X
    double adjustFactor = fabs(UTM2Img_.y/ UTM2Img_.x);
    double newScaleX = UTM2Img_.x*adjustFactor,
           newScaleY = UTM2Img_.y;
    cout << "New img. scale "
         << newScaleX << " , " << newScaleY << endl
         << "Adjust factor " << adjustFactor << endl;
    Mat normMap;
    resize(mapImg,normMap,Size(),adjustFactor,1.0);
    imwrite(format("NormMapRes_%lf.png",newScaleX),
            normMap);
  }

}

bool SatelliteManager::hasMap()
{
    return !mapImg.empty();
}

int SatelliteManager::cols()
{
    return mapImg.cols;
}

int SatelliteManager::rows()
{
    return mapImg.rows;
}
