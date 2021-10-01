#ifndef DATASETREADER_H
#define DATASETREADER_H

#include <string>
#include <vector>
#include <ostream>

using namespace std;

#include <opencv2/core/core.hpp>
using namespace cv;

#include <boost/filesystem.hpp>
using namespace boost::filesystem;

class DatasetFrame
{
public:
    int id;
    double timestamp;
    Point2d p, c;
    double heading; // Heading in degrees
    bool isFrameCarrected;

    DatasetFrame(int id, double timeStamp,
                const Point2d &p, const Point2d &c, double heading):
        id(id),timestamp(timeStamp),
        p(p),c(c),heading(heading){}
    DatasetFrame():
        id(-1),timestamp(-1),heading(999),
        isFrameCarrected(false){}
    bool operator < (const DatasetFrame &data) const
    { return (timestamp <  data.timestamp); }
};

ostream& operator << (ostream &os, const DatasetFrame &d);
ostream& operator << (ostream &os, const vector<DatasetFrame> &vd);

typedef struct
{
    double timeStamp;
    double x,y,z;
}CSVPosition;

typedef struct
{
    double timeStamp;
    string fileName;
}CSVFrame;

typedef struct
{
    double timeStamp;
    float heading;
}CSVHeading;

class DatasetReader
{
private:
     Point2d *cvPositions;
     string csvPositionName,
            csvFrameName,
            csvHeadingName;

     void deleteCVPoints();
     void findFirstTimeIntersectionData(int &positionId, int &headingId,
                                        int &frameId);
     void findLastTimeIntersectionData(int &positionId, int &headingId,
                                       int &frameId);

public:
    double sonarRange;

    DatasetReader();
    ~DatasetReader();

    vector<CSVPosition> csvPosition;
    vector<CSVFrame> csvFrames;
    vector<CSVHeading> csvHeading;

    bool loadDataset(path datasetPath, vector<DatasetFrame> &frames);

    bool loadSimplePositionFromCSV(path positionCSV);
    bool loadPositionFromCSV(path positionCSV); /** @todo - Need to be change to loadGPSPositionFromCSV **/
    bool loadFramesFromCSV(path frameInfo);
    bool loadHeadingFromCSV(path headingCSV, float magnectCorrection);

    void syncFramesData(vector<DatasetFrame> &frames);
    void syncFramesDataByPosition(vector<DatasetFrame> &frames, double maxDelay=1.0);

    void interporlateDataBySonImages(vector<DatasetFrame> &frames);

    bool writeSyncDatasetByPosition(path destPath, double maxDelay=1.0);

    void exclusiveSyncFramesData(vector<DatasetFrame> &frames,
                        double timeOffsetThreshold);

    Point2d *getCVPositions(unsigned *size);

    int findPositionAfter(double timeStamp);
    int findPositionBefore(double timeStamp);

    int findHeadingAfter(double timeStamp);
    int findHeadingBefore(double timeStamp);

    int findFrameAfter(double timeStamp);
    int findFrameBefore(double timeStamp);

};

#endif // DATASETREADER_H
