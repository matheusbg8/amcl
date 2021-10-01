#include "DatasetReader.h"
#include <cstdio>
#include <iostream>
#include <cmath>

#include "../CSVReader/CSVReader2.h"

using namespace std;

ostream &operator <<(ostream &os, const DatasetFrame &d)
{
    os << "id: " << d.id << " t: " << d.timestamp
       << " p: " << d.p << " h: " << d.heading << endl;
    return os;
}
ostream& operator << (ostream &os, const vector<DatasetFrame> &vd)
{
    for(unsigned i = 0 ; i < vd.size(); i++)
        os << vd[i];
    return os;
}


inline double linInterpole(double x0, double x1, double t0, double t1, double t)
{
    return x0 + (t-t0)/(t1-t0) * (x1-x0);
}

double linInterpole360Compass(double x0, double x1, double t0, double t1, double t)
{
    double diff1, diff2;

    diff2 = x1 - x0;
    if(x0 > x1)
        diff1 = 360.0 - x0 + x1;
    else
        diff1 = -(x0 + 360.0 - x1);

    diff1 = x0 + (t-t0)/(t1-t0) * (fabs(diff1) < fabs(diff2)? diff1 : diff2);
    if(diff1 > 360.0) diff1-=360.0;
    else if(diff1 < 0.0) diff1+= 360.0;

    return diff1;
}

void DatasetReader::deleteCVPoints()
{
    if(cvPositions != 0x0)
    {
        delete [] cvPositions;
    }
}

void DatasetReader::findFirstTimeIntersectionData(int &positionId, int &headingId,
                                                  int &frameId)
{
    positionId = headingId = frameId = 0;

    CSVPosition &p = csvPosition[0];
    CSVFrame &fr = csvFrames[0];
    CSVHeading &h = csvHeading[0];

    // Find reference sensor, the most recentely enabled sensor
    if(fr.timeStamp > p.timeStamp && fr.timeStamp > h.timeStamp)
    {
        // Frame is the reference
        frameId = 0;
        positionId = findPositionAfter(fr.timeStamp);
        headingId = findHeadingAfter(fr.timeStamp);
    }else if(p.timeStamp > fr.timeStamp && p.timeStamp > h.timeStamp)
    {
        // Position is the reference
        positionId = 0;
        frameId = findFrameAfter(p.timeStamp);
        headingId = findHeadingAfter(p.timeStamp);
    }else
    {
        // Heading is the reference
        headingId = 0;
        frameId = findFrameAfter(h.timeStamp);
        positionId = findPositionAfter(h.timeStamp);
    }
}

void DatasetReader::findLastTimeIntersectionData(int &positionId, int &headingId,
                                                 int &frameId)
{
    positionId = csvPosition.size()-1;
    headingId = csvHeading.size()-1;
    frameId = csvFrames.size()-1;

    CSVPosition &p = csvPosition[positionId];
    CSVFrame &fr = csvFrames[frameId];
    CSVHeading &h = csvHeading[headingId];

    // Find reference sensor, the most recentely disabled sensor
    if(fr.timeStamp < p.timeStamp && fr.timeStamp < h.timeStamp)
    {
        // Frame is the reference
        frameId = csvFrames.size()-1;
        positionId = findPositionBefore(fr.timeStamp);
        headingId = findHeadingBefore(fr.timeStamp);
    }else if(p.timeStamp < fr.timeStamp && p.timeStamp < h.timeStamp)
    {
        // Position is the reference
        positionId = csvPosition.size()-1;
        frameId = findFrameBefore(p.timeStamp);
        headingId = findHeadingBefore(p.timeStamp);
    }else
    {
        // Heading is the reference
        headingId = csvHeading.size()-1;
        frameId = findFrameBefore(h.timeStamp);
        positionId = findPositionBefore(h.timeStamp);
    }
}

DatasetReader::DatasetReader():
    cvPositions(0x0), sonarRange(50.f)
{

}

DatasetReader::~DatasetReader()
{
    deleteCVPoints();
}

bool DatasetReader::loadDataset(path datasetPath,vector<DatasetFrame> &frames)
{
    // Verify if directories exist
    if(!is_directory(datasetPath))
    {
        cout << "Incorrect input data: " << datasetPath.string()
             << " (must be a directory)" << endl;
        return false;
    }

    string position,
           heading,sonar;

    cout << "Loading metadata: "
         << (datasetPath / "dataset.yml").string()
         << endl;

    FileStorage fs((datasetPath / "dataset.yml").string(),FileStorage::READ);

    cout << "Checking files!" << endl;
    if(!fs.isOpened())
    {
      cout << "Could not open meta data: "
           << (datasetPath / "dataset.yml").string()
           << endl;
      return false;
    }

    fs["sonar"] >> sonar;
    cout << "Sonar data: " << sonar << endl;
    fs["heading"] >> heading;
    cout << "Heading data: " << heading << endl;
    fs["position"] >> position;
    cout << "Position data: " << position << endl;
    fs["sonarRange"] >> sonarRange;
    cout << "Sonar range: " << sonarRange << endl;

    fs.release();

    // Load the dataset
    bool ok=false;

    // Loading DGPS
    cout << "Loading position from CSV: "
         << (datasetPath / position).string()
         << endl;
    ok = loadPositionFromCSV( (datasetPath / position).string());

    // Loading Compass
    cout << "Loading compass from CSV: "
         << (datasetPath / heading).string()
         << endl;
    double headingCorrection=0.0;
    ok &= loadHeadingFromCSV ( (datasetPath / heading).string(),
                                   headingCorrection);

    // Loading Sonar Images
    ok &= loadFramesFromCSV  ( (datasetPath / sonar).string());
    cout << "Loading sonar Imgs from CSV: "
         << (datasetPath / sonar).string()
         << endl;

    // Syc information and create GTLoop frames
//  syncFramesData(frames);
//  exclusiveSyncFramesData(frames,0.15);
    interporlateDataBySonImages(frames);

//    cout << setprecision(12) << frames << endl;
    //    cout << m_frames << endl;
    return ok;
}

bool DatasetReader::loadSimplePositionFromCSV(path positionCSV)
{
    CSVReader2 csv;

    if(! csv.open(positionCSV.c_str(),4,',',
                  CSV_DOUBLE, // Time stamp
                  CSV_DOUBLE, // X (easting)
                  CSV_DOUBLE, // Y (northing)
                  CSV_DOUBLE // Z Altitude
                  ))
    {
        cout << "GTLoopLoader:: TF position file "
             << positionCSV.string()
             << " can't be loaded"
             << endl;
        return false;
    }

    string brand;
    int zone;
    CSVPosition p;

    csvPosition.clear();

    while(csv.read(&p.timeStamp, &p.x, &p.y,
                   &p.z, &brand,&zone))
    {
        csvPosition.push_back(p);
    }

    csv.close();
    csvPositionName = positionCSV.filename().string();

//    cout.precision(15);
//    for(unsigned i = 0; i < csvPosition.size(); i++)
//    {
//        ref = &(csvPosition[i]);
//        cout << ref->timeStamp << " , "
//             << ref->x << " , "
//             << ref->y << " , "
//             << ref->z << endl;
//    }
    return true;
}

/**
 * @brief GTLoopLoader::loadPositionFromCSV Load UTM position from
 * a CSV file. This file must be in the following format:
 * Six columns split by comma "," character where:
 * 1st - The time stamp in seconds (double precision value).
 * 2nd - The X (easting) in meters (double precision value).
 * 3th - The Y (northing) in meters (double precision value).
 * 4th - The Altitude in meters.
 * 5th - Brand leter (string value).
 * 6th - Zone number (integer value).
 * It is allowed to have header line, but it must be
 * the first line and start with character "#".
 * @param positionCSV - The path to the CSV file.
 * @return - true if the file was successfully loaded.
 */
bool DatasetReader::loadPositionFromCSV(path positionCSV)
{
    CSVReader2 csv;

    if(! csv.open(positionCSV.c_str(),6,',',
                  CSV_DOUBLE, // Time stamp
                  CSV_DOUBLE, // X (easting)
                  CSV_DOUBLE, // Y (northing)
                  CSV_DOUBLE, // Altitude
                  CSV_STRING, // Brand
                  CSV_INT // Zone
                  ))
    {
        cout << "GTLoopLoader:: TF position file "
             << positionCSV.string()
             << " can't be loaded"
             << endl;
        return false;
    }

    string brand;
    int zone;
    CSVPosition p;

    csvPosition.clear();

    while(csv.read(&p.timeStamp, &p.x, &p.y,
                   &p.z, &brand,&zone))
    {
        csvPosition.push_back(p);
    }

    csv.close();
    csvPositionName = positionCSV.filename().string();

//    cout.precision(15);
//    for(unsigned i = 0; i < csvPosition.size(); i++)
//    {
//        ref = &(csvPosition[i]);
//        cout << ref->timeStamp << " , "
//             << ref->x << " , "
//             << ref->y << " , "
//             << ref->z << endl;
//    }
    return true;
}

bool DatasetReader::loadFramesFromCSV(path frameInfo)
{
    CSVReader2 csv;

    if(! csv.open(frameInfo.c_str(),2,',',
                  CSV_DOUBLE, // Time stamp
                  CSV_STRING  // Image file name
                  ))
    {
        cout << "GTLoopLoader:: frame information file "
             << frameInfo.string()
             << " can't be loaded"
             << endl;
        return false;
    }

    csvFrames.clear();

    CSVFrame frame;

    while(csv.read(&frame.timeStamp,
                   &frame.fileName))
    {
        csvFrames.push_back(frame);
    }

    csv.close();
    csvFrameName = frameInfo.filename().string();

//    cout.precision(15);
//    for(unsigned i = 0; i < csvFrames.size(); i++)
//    {
//        cout << csvFrames[i].timeStamp << " , "
//             << csvFrames[i].fileName << endl;
//    }
    return true;
}

bool DatasetReader::loadHeadingFromCSV(path headingCSV, float magnectCorrection)
{
    CSVReader2 csv;

    if(! csv.open(headingCSV.c_str(),2,',',
                  CSV_DOUBLE, // Time stamp
                  CSV_FLOAT  // Heading in degrees
                  ))
    {
        cout << "GTLoopLoader:: frame information file "
             << headingCSV.string()
             << " can't be loaded"
             << endl;
        return false;
    }

    csvHeading.clear();
    CSVHeading heading;

    while(csv.read(&heading.timeStamp, &heading.heading))
    {
        // Magnetic correction to true north
        heading.heading += magnectCorrection;

        // Normalization 0 to 360
        if(heading.heading<0.f) heading.heading += 360.f;
        else if(heading.heading>360.f)
            heading.heading -= 360.f;

        csvHeading.push_back(heading);
    }

    csv.close();
    csvHeadingName = headingCSV.filename().string();

//    cout.precision(15);
//    for(unsigned i = 0; i < csvHeading.size(); i++)
//    {
//        cout << csvHeading[i].timeStamp << " , "
//             << csvHeading[i].heading << endl;
//    }

    return true;
}

/**
 * @brief
 *      Compute GTFLoop frames with position and heading.
 * @param frames - Frames that will bee filled
 */
void DatasetReader::syncFramesData(vector<DatasetFrame> &frames)
{
    unsigned headingId=0, positionId=0, frameId=0;
    double currentTime, nextTime, frameTime;

    frames.clear();
    frames.reserve(csvFrames.size()+1);

    // Sort all msgs by time

    for(frameId=0 ; frameId < csvFrames.size(); frameId++)
    {

        // Take next frame
        CSVFrame &fr = csvFrames[frameId];
        frameTime = fr.timeStamp;

        // Find closest Heading by timestamp
        currentTime = csvHeading[headingId].timeStamp;

        // While has next msgs
        while(headingId+1 < csvHeading.size())
        {
            nextTime = csvHeading[headingId+1].timeStamp;

            if(abs(nextTime - frameTime) < abs(currentTime - frameTime))
            {
                headingId++;
                currentTime = csvHeading[headingId].timeStamp;
            }else
                break; // current headingId is the closest
        }

        CSVHeading &heading = csvHeading[headingId];

        // Find closest Position by timestamp
        currentTime = csvPosition[positionId].timeStamp;
        // While has next msgs
        while(positionId+1 < csvPosition.size())
        {
            nextTime = csvPosition[positionId+1].timeStamp;
            if(abs(nextTime - frameTime) < abs(currentTime - frameTime))
            {
                positionId++;
                currentTime = csvPosition[positionId].timeStamp;
            }else
                break; // current positionId is the closest
        }

        CSVPosition &position = csvPosition[positionId];

        double radAng = heading.heading*M_PI/180.0;
        Point2d p(position.x, position.y),
                direction( sin(radAng),cos(radAng) );

        frames.push_back(DatasetFrame(frameId, fr.timeStamp,
                                     p,
                                     p + direction*sonarRange*0.5,
                                     heading.heading));

    }

//    cout << "Frames:" << endl;
//    for(unsigned i = 0 ; i < frames.size(); i++)
//    {
//        cout << frames[i].fileName << ", "
//             << frames[i].p << ", "
//             << frames[i].c << ", "
//             << frames[i].heading << endl;
    //    }
}

void DatasetReader::syncFramesDataByPosition(vector<DatasetFrame> &frames, double maxDelay)
{
    unsigned headingId=0, positionId=0, frameId=0;
    double currentTime, nextTime, positionTime;

    frames.clear();
    frames.reserve(csvPosition.size()+1);

    // Sort all msgs by time

    for(positionId=0 ; positionId < csvPosition.size(); positionId++)
    {

        // Take next position
        CSVPosition &position = csvPosition[positionId];
        positionTime = position.timeStamp;

        // Find closest Heading by current timestamp
        // get current heading time
        currentTime = csvHeading[headingId].timeStamp;

        // While has next msgs
        while(headingId+1 < csvHeading.size())
        {
            // get next heading time
            nextTime = csvHeading[headingId+1].timeStamp;

            if(fabs(nextTime - positionTime) <= fabs(currentTime - positionTime))
            {
                headingId++;
                currentTime = csvHeading[headingId].timeStamp;
            }else
                break; // current headingId is the closest
        }
        if(fabs(currentTime - positionTime) > maxDelay) continue;

        CSVHeading &heading = csvHeading[headingId];

        // Find closest frame by timestamp
        currentTime = csvFrames[frameId].timeStamp;
        // While has next msgs
        while(frameId+1 < csvFrames.size())
        {
            nextTime = csvFrames[frameId+1].timeStamp;
            if(fabs(nextTime - positionTime) <= fabs(currentTime - positionTime))
            {
                frameId++;
                currentTime = csvFrames[frameId].timeStamp;
            }else
                break; // current positionId is the closest
        }
        if(fabs(currentTime - positionTime) > maxDelay) continue;

//        CSVFrame &frame = csvFrames[frameId];

//        cout << frame.fileName << endl;

        double radAng = heading.heading*M_PI/180.0;
        Point2d p(position.x, position.y),
                direction( sin(radAng),cos(radAng) );

        frames.push_back(DatasetFrame(frameId, position.timeStamp,
                                     p,
                                     p + direction*sonarRange*0.5,
                                     heading.heading));
    }

//    cout << "Frames:" << endl;
//    for(unsigned i = 0 ; i < frames.size(); i++)
//    {
//        cout << frames[i].fileName << ", "
//             << frames[i].p << ", "
//             << frames[i].c << ", "
//             << frames[i].heading << endl;
//    }

}

/**
 * @brief DatasetReader::interporlateDataBySonImages use linear interpolation
 * to create heading and position for all sonar images.
 * @param frames - Computed frames.
 */
void DatasetReader::interporlateDataBySonImages(vector<DatasetFrame> &frames)
{
    if(csvPosition.size() < 2 || csvHeading.size() < 2)
    {
        cout << "DatasetReader::interporlateDataBySonImages - No enough data!" << endl;
        return;
    }

    // Find intersection interval
    int endHId, endPId, endFrId,
        headingId=0, positionId=0, frameId=0;

    findFirstTimeIntersectionData(positionId,headingId,frameId);
    findLastTimeIntersectionData(endPId,endHId,endFrId);

    if(positionId< 0 || headingId < 0 || frameId < 0 ||
       endPId < 0 || endHId < 0 || endFrId < 0)
    {
        cout << "DatasetReader::interporlateDataBySonImages - No data time intersection!" << endl;
        return;
    }

    double currentTime, referenceTime;

    frames.clear();
    frames.reserve(endFrId-frameId+1);

    // Sort all msgs by time
    // We are supposing they already are ordered
    for(frameId ; frameId <= endFrId; frameId++)
    {
        DatasetFrame newFr;

        // Get the time of the current frame (That is a sonar image)
        referenceTime = csvFrames[frameId].timeStamp;
        newFr.timestamp = referenceTime;
        newFr.id =frameId;

        // Find the two closest timestamp in position data
        currentTime = csvPosition[positionId].timeStamp;

        // While has next msgs
        while(currentTime < referenceTime && positionId < endPId)
        {
            positionId++;
            currentTime = csvPosition[positionId].timeStamp;
        }

        // Now positionId-1 and positionId are the closer data of referenceTime
        {
            if(positionId == 0) positionId++;
            CSVPosition &prev = csvPosition[positionId-1],
                        &next = csvPosition[positionId];

            newFr.p = Point2d(
                linInterpole(prev.x,next.x,prev.timeStamp,next.timeStamp,referenceTime),
                linInterpole(prev.y,next.y,prev.timeStamp,next.timeStamp,referenceTime));
        }

        // Repeat the same process for heading data

        // Find the two closest timestamp in heading data
        currentTime = csvHeading[headingId].timeStamp;

        // While has next msgs
        while(currentTime < referenceTime && headingId < endHId)
        {
            headingId++;
            currentTime = csvHeading[headingId].timeStamp;
        }

        // Now headingId-1 and headingId are the closer data of referenceTime
        {
            if(headingId == 0) headingId++;
            CSVHeading &prev = csvHeading[headingId-1],
                       &next = csvHeading[headingId];

            double heading = linInterpole360Compass(prev.heading, next.heading,prev.timeStamp,next.timeStamp,referenceTime);

            double radAng = heading*M_PI/180.0;
            Point2d direction( sin(radAng),cos(radAng) );
            newFr.c = newFr.p + direction*sonarRange*0.5;
            newFr.heading = heading;
        }

        frames.push_back(newFr);
    }
}

bool DatasetReader::writeSyncDatasetByPosition(path destPath, double maxDelay)
{
    if(!is_directory(destPath) && !create_directory(destPath))
    {
        cout << "DatasetReader::writeSyncDatasetByPosition - Error, it wasn't"
                " possible to create directory " << destPath.string() << endl;
        return false;
    }

    setlocale (LC_ALL,"C");
    FILE *fHeading = fopen((destPath/csvHeadingName).c_str(),"w"),
         *fPosition = fopen((destPath/csvPositionName).c_str(),"w"),
         *fFrame = fopen((destPath/csvFrameName).c_str(),"w"),
         *fRawFrameNames = fopen((destPath/"Frames.txt").c_str(),"w");

    if(!fHeading || !fPosition || !fFrame)
    {
        cout << "DatasetReader::writeSyncDatasetByPosition - Error, it wasn't"
                " possible to create data files on directory " << destPath.string() << endl;
        return false;
    }

    // Write csv headers

    fprintf(fHeading, "#time stamp since jan 1st 1970 in seconds, heading in degress from 0 to 360\n");
    fprintf(fPosition, "#time stamp since jan 1st 1970 in seconds,"
                       "X (easting) in meters, Y(northing) in meters, Z in meters (altitude or depth)\n");
    fprintf(fFrame, "#time stamp since jan 1st 1970 in seconds,"
                       "frame file name\n");


    unsigned headingId=0, positionId=0, frameId=0;
    double currentTime, nextTime, positionTime;

    // Sort all msgs by time

    for(positionId=0 ; positionId < csvPosition.size(); positionId++)
    {
        // Take next position
        CSVPosition &position = csvPosition[positionId];
        positionTime = position.timeStamp;

        // Find closest Heading by current timestamp
        // get current heading time
        currentTime = csvHeading[headingId].timeStamp;

        // While has next msgs
        while(headingId+1 < csvHeading.size())
        {
            // get next heading time
            nextTime = csvHeading[headingId+1].timeStamp;

            if(fabs(nextTime - positionTime) <= fabs(currentTime - positionTime))
            {
                headingId++;
                currentTime = csvHeading[headingId].timeStamp;
            }else
                break; // current headingId is the closest
        }
        if(fabs(currentTime - positionTime) > maxDelay) continue;

        CSVHeading &heading = csvHeading[headingId];

        // Find closest frame by timestamp
        currentTime = csvFrames[frameId].timeStamp;
        // While has next msgs
        while(frameId+1 < csvFrames.size())
        {
            nextTime = csvFrames[frameId+1].timeStamp;
            if(fabs(nextTime - positionTime) <= fabs(currentTime - positionTime))
            {
                frameId++;
                currentTime = csvFrames[frameId].timeStamp;
            }else
                break; // current positionId is the closest
        }
        if(fabs(currentTime - positionTime) > maxDelay) continue;

        CSVFrame &frame = csvFrames[frameId];

        // Write new data
        fprintf(fHeading, "%.14lg,%.4g\n",
                heading.timeStamp,heading.heading);

        fprintf(fPosition, "%.14lg,%.14lg,%.14lg,%.14lg\n",position.timeStamp,
                position.x,position.y,position.z);

        fprintf(fFrame, "%.14lg,%s\n",
                frame.timeStamp, frame.fileName.c_str());

        fprintf(fRawFrameNames, "%s\n",
                frame.fileName.c_str());
    }

    // Close files
    fclose(fPosition);
    fclose(fHeading);
    fclose(fFrame);
    fclose(fRawFrameNames);

//    cout << "Frames:" << endl;
//    for(unsigned i = 0 ; i < frames.size(); i++)
//    {
//        cout << frames[i].fileName << ", "
//             << frames[i].p << ", "
//             << frames[i].c << ", "
//             << frames[i].heading << endl;
//    }
    return true;
}

void DatasetReader::exclusiveSyncFramesData(vector<DatasetFrame> &frames,
                                           double timeOffsetThreshold)
{
    unsigned headingId=0, positionId=0, frameId=0;
    double currentTime, nextTime, frameTime;

    frames.reserve(csvFrames.size()+1);

    // Sort all msgs by time

    // It is supposed already ordered

    for(frameId=0; frameId < csvFrames.size(); frameId++)
    {
        // Take next frame
        CSVFrame &fr = csvFrames[frameId];
        frameTime = fr.timeStamp;

        // Find closest Heading by timestamp
        currentTime = csvHeading[headingId].timeStamp;
        // While has next msgs
        while(headingId+1 < csvHeading.size())
        {
            nextTime = csvHeading[headingId+1].timeStamp;

            if(fabs(nextTime - frameTime) <= fabs(currentTime - frameTime))
            {
                headingId++;
                currentTime = csvHeading[headingId].timeStamp;
            }else
                break; // current headingId is the closest
        }

        // Apply the time difference threshold
        if(fabs(currentTime - frameTime) > timeOffsetThreshold)
            continue; // Ignore this frame and go to the next

        CSVHeading &heading = csvHeading[headingId];

        // Find closest Position by timestamp
        currentTime = csvPosition[positionId].timeStamp;
        // While has next msgs
        while(positionId+1 < csvPosition.size())
        {
            nextTime = csvPosition[positionId+1].timeStamp;
            if(fabs(nextTime - frameTime) <= fabs(currentTime - frameTime))
            {
                positionId++;
                currentTime = csvPosition[positionId].timeStamp;
            }else
                break; // current positionId is the closest
        }

        // Apply the time difference threshold
        if(fabs(currentTime - frameTime) > timeOffsetThreshold)
            continue; // Ignore this frame and go to the next

        CSVPosition &position = csvPosition[positionId];

        double radAng = heading.heading*M_PI/180.0;
        Point2f p(position.x, position.y),
                direction( sin(radAng),cos(radAng) );

        frames.push_back(DatasetFrame(frameId, fr.timeStamp,
                                     p,
                                     p + direction*sonarRange*0.5,
                                     heading.heading));
    }

//    cout << "Frames:" << endl;
//    for(unsigned i = 0 ; i < frames.size(); i++)
//    {
//        cout << frames[i].fileName << ", "
//             << frames[i].p << ", "
//             << frames[i].c << ", "
//             << frames[i].heading << endl;
    //    }
}

Point2d *DatasetReader::getCVPositions(unsigned *size)
{
    deleteCVPoints();

    cvPositions = new Point2d[csvPosition.size()];
    for(unsigned i = 0 ;i < csvPosition.size();i++)
    {
        cvPositions[i] = Point2d(csvPosition[i].x,csvPosition[i].y);
    }
    *size = csvPosition.size();
    return cvPositions;
}

/**
 * @brief DatasetReader::findPositionAfter - Return vector position
 * of first data aquired after a giving time.
 * @param timeStamp - Timestamp evaluated
 * @return vetor position of csvPostion or -1 if there are no data
 * after timeStamp
 */
int DatasetReader::findPositionAfter(double timeStamp)
{
    for(int i = 0 ; i < csvPosition.size(); i++)
        if(csvPosition[i].timeStamp > timeStamp)
            return i;
    return -1;
}

int DatasetReader::findPositionBefore(double timeStamp)
{
    for(int i = csvPosition.size()-1; i >=0 ; i--)
        if(csvPosition[i].timeStamp < timeStamp)
            return i;
    return -1;
}

int DatasetReader::findHeadingAfter(double timeStamp)
{
    for(int i = 0 ; i < csvHeading.size(); i++)
        if(csvHeading[i].timeStamp > timeStamp)
            return i;
    return -1;
}

int DatasetReader::findHeadingBefore(double timeStamp)
{
    for(int i = csvHeading.size()-1; i >=0 ; i--)
        if(csvHeading[i].timeStamp < timeStamp)
            return i;
    return -1;
}

int DatasetReader::findFrameAfter(double timeStamp)
{
    for(int i = 0 ; i < csvFrames.size(); i++)
        if(csvFrames[i].timeStamp > timeStamp)
            return i;
    return -1;
}

int DatasetReader::findFrameBefore(double timeStamp)
{
    for(int i = csvFrames.size()-1; i >=0 ; i--)
        if(csvFrames[i].timeStamp < timeStamp)
            return i;
    return -1;
}

