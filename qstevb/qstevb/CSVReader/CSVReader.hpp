
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

    // 写出时对字段做 CSV 转义：含逗号/制表符/引号/换行的字段用双引号包裹，内部引号翻倍
    string escapeItem(const string& item);
    
public:
    bool readFromFile(const char* filename);
    bool readFromData(const char* str_data, int len);
    
    CSVReader();
    CSVReader(const char* filename);
    
    void clear();

    const char* getItem(int row, int col);

    // 修改指定行、列的单元格（越界返回 false，不修改）
    bool setItem(int row, int col, const char* value);

    // 修改指定行、列的单元格（数值等类型，自动转字符串）
    template<class T>
    bool setItem(int row, int col, const T& value) {
        stringstream ss;
        ss << value;
        return setItem(row, col, ss.str().c_str());
    }

    // 将当前数据保存到 CSV 文件（覆盖写入，自动处理含逗号/引号字段的转义）
    bool saveToFile(const char* filename);
    
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