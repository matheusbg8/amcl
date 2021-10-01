#ifndef SATELLITEMANAGER_H
#define SATELLITEMANAGER_H

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <boost/filesystem.hpp>

using namespace cv;
using namespace std;
using namespace boost::filesystem;

class SatelliteManager
{
public:
    // ====== Map conversions =====
    Point2d imgRef[2], /// Two reference points in pixels
        UTMRef[2], /// The same two reference points in UTM
        UTM2Img_; /// The scale factor to transform UTM to pixel (pixel per meters)

    double UTM2ImgDiffXToY;

    Mat mapImg; /// Current satellite image

    void getSonarPolyOnImg(const Point2d &p, double heading, Point *poly,
                     unsigned nPoint=13, double range=50.0, double bearing=130.0) const;

    string mapName;

    SatelliteManager();

    SatelliteManager(const string &mapPath);

    void changeMapResolution(double resolutionFactor);
    void adjustResolution(double metersPerPixel);

    // Map methods
    bool loadMap(const path &mapPath);
    Point2d UTM2Img(const Point2d &UTMp) const;
    Point2d Img2UTM(const Point2d &ImgP) const;
    void UTM2Img(Point2d *UTMpts, unsigned nPts);

    void removeOffset();

    void parse2D_2_Point2d(const string &str, Point2d &p);

    // Map info query
    void minMaxUTMValues(Point2d &min, Point2d &max);
    void minMaxImgValues(Point2d &min, Point2d &max);
    void getWidthHeight(int *maxX, int *maxY);

    // Internal map operations
    void cropOnPoints(const vector<Point2d> &utmPoints,
                      double outSpaceInMeters);

    void cropOnPoints(const Point2d *utmPoints, unsigned nPts,
                      double outSpaceInMeters);

    // Operations with result
    Mat cropRectUTM(Point2f &utmP,
                 float heightInMeters, float withInMeters,
                 float orientationInDegrees);

    Mat cropRect(Point2f &imgP,
                 float height, float width,
                 float oriDeeg);

    Mat cropRect(Point2f &imgP,
                 float height, float width);

    Mat cropSonarFoV(Point2d p, double headingDegrees,
                    double sonarRange, double FoV,
                    bool doPositionAdjustment=false) const;


    Rect getRectThatFitsIntoImg(const Rect &r,
                                const Mat &im,
                                int &top, int &bottom,
                                int &left, int &right) const;

    bool getPixel(const Point2d &UTMPp,
                  Vec3b &pixel) const;
    Mat getMapImg();

    void setMapWhite();

    void normalizeMapAndSave();

    // Query operations
    bool hasMap();
    int cols();
    int rows();

};

#endif // SATELLITEMANAGER_H
