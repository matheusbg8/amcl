#include "CSVReader2.h"
#include "CSVData.h"
#include "DataExtractorUtilities.h"
#include <cstring>
#include <string>

CSVReader2::CSVReader2():
    csvFile(0x0),spliter('\0')
{

}

CSVReader2::~CSVReader2()
{
    close();
}

bool CSVReader2::open(const char *fileName, unsigned nCols, const char spliter, ...)
{
    // Open FILE
    csvFile = fopen(fileName,"r");
    if(csvFile == 0x0) return false;
    csvFileName = fileName;

    this->spliter = spliter;

    // Read field types definition
    types.reserve(nCols);
    va_list vl;
    va_start(vl,spliter);
    for(unsigned i=0;i<nCols;i++)
    {
        CSV_TYPE t = (CSV_TYPE) va_arg(vl,int); // Passing int because warnnings
        types.push_back(t);
    }
    va_end(vl);

    // Clean pointer addresses
    dataFieldAdress.clear();
    dataFieldAdress.resize(types.size(),0x0);

    return true;
}

void CSVReader2::close()
{
    if(csvFile!= 0x0)
    {
        fclose(csvFile);
        csvFile = 0x0;
        types.clear();
        dataFieldAdress.clear();
        csvFileName.clear();
        spliter = '\0';
    }
}

void CSVReader2::ignoreNextLine()
{
    // DataExtractorUtilities ignoreLine
    ignoreLine(csvFile);
}

/**
 * @brief CSVReader2::read read a CVS file
 * @param field1 - Each reading field
 * @return - true if it's OK or false when file end.
 */
bool CSVReader2::read(void *field1,...)
{
    if(csvFile == 0x0) return false;
    setlocale(LC_NUMERIC, "C");

    // Get data adress of each field
    if(types[0] != CSV_IGNORE)
        dataFieldAdress[0] = field1;

    // Get data adress
    va_list vl;
    va_start(vl,field1);
    for(unsigned i=1;i<dataFieldAdress.size();i++)
    {
        if(types[i] != CSV_IGNORE)
            dataFieldAdress[i] = va_arg(vl,void*);
    }
    va_end(vl);

    // Read control
    char field[1000];
    bool hasMsg = false;

    // Ignore comment lines
    while(isCommentLine(csvFile))
        ignoreLine(csvFile);

    // Read CSV line
    for(unsigned fieldID = 0;
        fieldID < dataFieldAdress.size() &&
       (hasMsg = extractField(csvFile,field,spliter) != -1);
        fieldID++)
    {
        // Interpret field
        switch(types[fieldID])
        {
            case CSV_STRING:
                *((string*) dataFieldAdress[fieldID]) = field;
            break;
            case CSV_INT:
                sscanf(field,"%d", (int*) dataFieldAdress[fieldID]);
            break;
            case CSV_LLU:
                sscanf(field,"%llu", (long long unsigned int*) dataFieldAdress[fieldID]);
            break;
            case CSV_FLOAT:
                sscanf(field,"%f", (float*) dataFieldAdress[fieldID]);
            break;
            case CSV_DOUBLE:
                sscanf(field,"%lf", (double*) dataFieldAdress[fieldID]);
            break;
            case CSV_INVALID:
            default:
                // Nothing to do
            break;
        }
    }
    return hasMsg;
}
