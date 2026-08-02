//
//  CSVReader.cpp
//  Test
//
//  Created by dong on 2017/5/4.
//  Copyright © 2017年 dong. All rights reserved.
//
#include "stdafx.h"
#include "CSVReader.hpp"
#include <assert.h>


CSVReader::CSVReader() {
    
}

CSVReader::CSVReader(const char* filename) {
    readFromFile(filename);
}

void CSVReader::clear() {
    data.clear();
}

const char* CSVReader::getItem(int row, int col) {
    if (row >= data.size()) {
        return nullptr;
    }
    
    if (col >= data[row].size()) {
        return nullptr;
    }
    
    return data[row][col].c_str();
}

long CSVReader::rowsNum() {
    return data.size();
}

long CSVReader::colsNum() {
    if (data.empty()) {
        return 0;
    }
    return data[0].size();
}

bool CSVReader::readFromFile(const char* filename) {
    clear();
    
    FILE* file = fopen(filename, "rt");
    if (!file) {
        return false;
    }
    
    char ch;
    
    /** 上一个字符是否\\r */
    bool r = false;
    
    string row;
    vector<string> row_vec;
    
    int i = 0;
    
    while ((ch = fgetc(file)) != EOF) {
        if (i < 3) {
            if (ch == '\357' || ch == '\273' || ch == '\277') {
                // 处理带BOM的UTF8文件
                i += 1;
                continue;
            }
            i += 1;
        }
        
        handleChar(ch, row, row_vec, r);
    }
    
    fclose(file);
    
    if (!row.empty()) {
        // 最后一行
        parseRow(row.c_str(), row_vec);
        data.push_back(row_vec);
    }
    
    return true;
}


bool CSVReader::readFromData(const char *str_data, int len) {
    clear();
    
    /** 上一个字符是否\\r */
    bool r = false;
    
    string row;
    vector<string> row_vec;
    
    // long len = strlen(str_data);
    
    for (int i = 0; i < len; i++) {
        
        const char& ch = str_data[i];
        
        if (i < 3) {
            if (ch == '\357' || ch == '\273' || ch == '\277') {
                // 处理带BOM的UTF8文件
                continue;
            }
        }
        
        handleChar(ch, row, row_vec, r);
    }
    
    if (!row.empty()) {
        // 最后一行
        parseRow(row.c_str(), row_vec);
        data.push_back(row_vec);
    }
    
    return true;
}

void CSVReader::handleChar(const char& ch, string& row, vector<string>& row_vec, bool& r) {
    if (ch == '\r') {
        r = true;
        
        // 一行的结束
        if (parseRow(row.c_str(), row_vec)) {
            data.push_back(row_vec);
            row = "";
            row_vec.clear();
        }
        
    } else {
        
        if (ch == '\n') {
            if (!r) {
                // 一行的结束
                if (parseRow(row.c_str(), row_vec)) {
                    data.push_back(row_vec);
                    row = "";
                    row_vec.clear();
                }
            }
            
        } else {
            row += ch;
        }
        
        r = false;
    }
}

bool CSVReader::parseRow(const char* row, vector<string>& result) {
    string str_row = trim(row);
    row = str_row.c_str();
    
    string str = "";
    
    /** 双引号开始 */
    bool quotes_begin = false;
    
    /** 一项开始了 */
    bool item_started = false;
    
    
    
    long len = strlen(row);
    
    auto isItemEnd = [&](int i) -> bool {
        if (i >= len) {
            return true;
        }
        if (row[i] == ',') {
            return true;
        }
        return false;
    };
    
    auto isQuotes = [&](int i) -> bool {
        if (i >= len) {
            return false;
        }
        if (row[i] == '"') {
            return true;
        }
        return false;
    };
    
    
    for (int i = 0; i < len; i++) {
        char ch = row[i];
        
        
        if (ch == ',' || ch == '\t') {
            if (quotes_begin) {
                // 这一项的开始是引号开始的，并且还没到结束的引号
                str += ch;
                
            } else {
                // 一项的结束
                result.push_back(str);
                str = "";
                item_started = false;
            }
            
        } else if (ch == '"') {
            // 双引号要不就是一项的开头和结尾，要不就是连续两个一起出现，否则都是错误的格式
            
            if (item_started) {
                
                bool item_end = false;
                
                if (quotes_begin) {
                    if (isItemEnd(i + 1)) {
                        
                        quotes_begin = false;
                        i += 1;
                        
                        // 一项的结束
                        result.push_back(str);
                        str = "";
                        item_started = false;
                        
                        item_end = true;
                    }
                }
                
                if (!item_end) {
                    
                    if (isQuotes(i + 1)) {
                        // 连续两个双引号
                        str += ch;
                        i += 1;
                    } else {
                        // 不合法的格式，内容里的双引号应该使用连续两个来表示
                        printf("%s\n", row);
                        assert(0);
                    }
                }
                
            } else {
                quotes_begin = true;
                item_started = true;
            }
            
        } else {
            item_started = true;
            str += ch;
        }
        
    }
    
    if (item_started) {
        // 最后一项
        if (quotes_begin) {
            // 没有结束引号，格式不对
            printf("%s\n", row);
            assert(0);
        }
        
        result.push_back(str);
    }
    
    return true;
}

string CSVReader::lTrim(const string& str)
{
    if (str.compare("") == 0) {
        return "";
    }
    return str.substr(str.find_first_not_of(" \n\r\t"));
}

string CSVReader::rTrim(const string& str)
{
    if (str.compare("") == 0) {
        return "";
    }
    return str.substr(0, str.find_last_not_of(" \n\r\t") + 1);
}

string CSVReader::trim(const string& str)
{
    return lTrim(rTrim(str));
}