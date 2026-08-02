
#ifndef CSVReader_hpp
#define CSVReader_hpp

#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>

using namespace std;

class CSVReader {
    
private:
    vector< vector<string> > data;
    
    string lTrim(const string& str);
    string rTrim(const string& str);
    string trim(const string& str);
    
    void handleChar(const char& ch, string& row, vector<string>& row_vec, bool& r);
    bool parseRow(const char* row, vector<string>& result);
    
public:
    bool readFromFile(const char* filename);
    bool readFromData(const char* str_data, int len);
    
    CSVReader();
    CSVReader(const char* filename);
    
    void clear();
    
    const char* getItem(int row, int col);
    
    template<class T>
    void copyItem(T& t, int row, int col) {
        const char* value = getItem(row, col);
        if (!value) {
            return;
        }
        
        string d = value;
        
        stringstream ss;
        ss << d;
        ss >> t;
    }
    
    long rowsNum();
    long colsNum();
};

#endif /* CSVReader_hpp */