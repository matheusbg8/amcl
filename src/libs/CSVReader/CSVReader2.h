#ifndef CSVREADER2_H
#define CSVREADER2_H

#include <vector>
#include <cstdarg>
#include <cstdio>
#include <string>

#include "CSVReader.h"

using namespace std;

class CSVReader2
{
protected:
    FILE *csvFile;
    vector<CSV_TYPE> types;
    vector<void*> dataFieldAdress;
    string csvFileName;
    char spliter;
public:
    CSVReader2();
    ~CSVReader2();

    void ignoreNextLine();

    bool open(const char *fileName, unsigned nCols, const char spliter, ...);
    void close();

    bool read(void *field1,...);
};

#endif // CSVREADER2_H
