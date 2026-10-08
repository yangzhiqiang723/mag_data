/************************************************************************
 * QPlotRunner.h  --  MFC / Win32 调用 qplot.exe 的轻量封装 (header-only)
 *
 * 只需 <windows.h> + <string>, 无需 ATL.
 *
 * 用法 (Unicode MFC 工程, 最常见):
 *
 *   #include "QPlotRunner.h"
 *
 *   QPlotResult r;
 *   CStringW args = L"-f \"D:\\data\\1.csv\" -m col -c 0,1,2 "
 *                  L"-o \"D:\\data\\1.png\"";
 *   if (QPlotRun(L"D:\\tools\\qplot.exe", args, r, 60000) && r.ok)
 *   {
 *       ShowPng(r.imagePath.c_str(), IDC_PIC);   // r.imagePath = png 绝对路径
 *   }
 *   else
 *   {
 *       AfxMessageBox(CString(L"失败: ") + r.message.c_str());
 *   }
 *
 * 多字节字符集工程 (Properties -> Character Set = "Multi-Byte") 用 QPlotRunA,
 *   它按系统 ANSI 代码页 (一般 GBK) 转码. 一般建议先在工程设置里改成 Unicode.
 ************************************************************************/
#pragma once
#include <windows.h>
#include <string>

//------------------------------------------------------------------ 结果
struct QPlotResult
{
    bool        ok;          // 收到 "OK " 且退出码 0
    DWORD       exitCode;
    std::wstring text;       // exe 完整输出 (UTF-8 -> Unicode)
    std::wstring message;    // 失败原因
    std::wstring imagePath;  // 成功时: 图片绝对路径; noise 模式: 标准差字符串
    QPlotResult() : ok(false), exitCode(0) {}
};

//------------------------------------------------ UTF-8 / ANSI -> Unicode
inline std::wstring QUtf8ToWide(const char* s, size_t len)
{
    if (!s || len == 0) return std::wstring();
    int n = MultiByteToWideChar(CP_UTF8, 0, s, (int)len, NULL, 0);
    if (n <= 0) return std::wstring();
    std::wstring out;
    out.resize((size_t)n);
    MultiByteToWideChar(CP_UTF8, 0, s, (int)len, &out[0], n);
    return out;
}

inline std::wstring QAcpToWide(const char* s)
{
    if (!s || !*s) return std::wstring();
    int n = MultiByteToWideChar(CP_ACP, 0, s, -1, NULL, 0);
    if (n <= 0) return std::wstring();
    std::wstring out;
    out.resize((size_t)n);
    if (MultiByteToWideChar(CP_ACP, 0, s, -1, &out[0], n) > 0)
    {
        if (!out.empty() && out.back() == L'\0') out.pop_back();
    }
    return out;
}

//------------------------------------------------------------ 主调用
inline bool QPlotRun(const wchar_t* exePath, const wchar_t* args,
                     QPlotResult& result, DWORD timeoutMs = 60000)
{
    result = QPlotResult();

    // 1) 匿名管道 (句柄可继承)
    SECURITY_ATTRIBUTES sa = { sizeof(SECURITY_ATTRIBUTES), NULL, TRUE };
    HANDLE hRead = NULL, hWrite = NULL;
    if (!CreatePipe(&hRead, &hWrite, &sa, 0))
    {
        result.message = L"CreatePipe 失败";
        return false;
    }

    // 2) 启动子进程: 隐藏窗口 + 重定向 stdout/stderr
    STARTUPINFOW si = { 0 };
    si.cb         = sizeof(si);
    si.dwFlags    = STARTF_USESTDHANDLES | STARTF_USESHOWWINDOW;
    si.hStdOutput = hWrite;
    si.hStdError  = hWrite;
    si.hStdInput  = NULL;
    si.wShowWindow = SW_HIDE;

    PROCESS_INFORMATION pi = { 0 };

    std::wstring cmd;
    cmd.reserve(wcslen(exePath ? exePath : L"") +
                wcslen(args ? args : L"") + 16);
    cmd  = L"\"";
    cmd += exePath ? exePath : L"";
    cmd += L"\" ";
    cmd += args ? args : L"";

    BOOL started = CreateProcessW(
        NULL, &cmd[0], NULL, NULL, TRUE,
        CREATE_NO_WINDOW | NORMAL_PRIORITY_CLASS,
        NULL, NULL, &si, &pi);

    if (!started)
    {
        DWORD err = GetLastError();
        CloseHandle(hRead);
        CloseHandle(hWrite);
        result.message = L"无法启动 qplot.exe, 错误码 " + std::to_wstring(err);
        return false;
    }

    // 父进程必须关掉写端, 否则 ReadFile 永远等不到 EOF
    CloseHandle(hWrite);
    hWrite = NULL;

    // 3) 边等边读 (避免管道缓冲满阻塞子进程)
    std::string raw;
    char  tmp[4096];
    DWORD got = 0;
    DWORD t0  = GetTickCount();
    bool  finished = false;

    while (true)
    {
        DWORD avail = 0;
        if (PeekNamedPipe(hRead, NULL, 0, NULL, &avail, NULL) && avail > 0)
        {
            if (ReadFile(hRead, tmp, sizeof(tmp), &got, NULL) && got > 0)
                raw.append(tmp, got);
            continue;
        }

        if (WaitForSingleObject(pi.hProcess, 50) == WAIT_OBJECT_0)
        {
            // 进程结束, 把管道残余读干净
            while (ReadFile(hRead, tmp, sizeof(tmp), &got, NULL) && got > 0)
                raw.append(tmp, got);
            finished = true;
            break;
        }

        if (GetTickCount() - t0 > timeoutMs)
        {
            TerminateProcess(pi.hProcess, 3);
            result.message = L"qplot.exe 执行超时";
            break;
        }
    }

    DWORD code = 0;
    if (finished) GetExitCodeProcess(pi.hProcess, &code);

    CloseHandle(pi.hThread);
    CloseHandle(pi.hProcess);
    CloseHandle(hRead);

    result.exitCode = code;
    result.text     = QUtf8ToWide(raw.data(), raw.size());

    // 4) 解析最后一行 "OK ..." / "ERROR ..."
    std::wstring line;
    size_t pos = result.text.find_last_of(L'\n');
    line = (pos == std::wstring::npos) ? result.text : result.text.substr(pos + 1);
    while (!line.empty() && (line.back() == L'\r' || line.back() == L'\n'))
        line.pop_back();

    if (line.size() >= 3 && line.compare(0, 3, L"OK ") == 0)
    {
        result.ok        = true;
        result.imagePath = line.substr(3);
    }
    else if (line.size() >= 6 && line.compare(0, 6, L"ERROR ") == 0)
    {
        result.message = line.substr(6);
    }
    else
    {
        result.message = line.empty() ? L"未知错误 (无输出)" : line;
    }
    return true;
}

//--------------------------------------- 多字节工程版: 按系统 ANSI 代码页
inline bool QPlotRunA(const char* exePath, const char* args,
                      QPlotResult& result, DWORD timeoutMs = 60000)
{
    std::wstring wExe = QAcpToWide(exePath);
    std::wstring wArgs = QAcpToWide(args);
    return QPlotRun(wExe.c_str(), wArgs.c_str(), result, timeoutMs);
}

//--------------------------------------- 异步版: 触发后立即返回
// 适合批量调用. exe 画完自己退出, 结果直接写在 -o 指定路径上.
inline bool QPlotRunAsync(const wchar_t* exePath, const wchar_t* args)
{
    std::wstring cmd;
    cmd = L"\"";
    cmd += exePath ? exePath : L"";
    cmd += L"\" ";
    cmd += args ? args : L"";

    STARTUPINFOW si = { 0 };
    si.cb = sizeof(si);
    si.dwFlags = STARTF_USESHOWWINDOW;
    si.wShowWindow = SW_HIDE;

    PROCESS_INFORMATION pi = { 0 };
    BOOL ok = CreateProcessW(NULL, &cmd[0], NULL, NULL, FALSE,
                             CREATE_NO_WINDOW | NORMAL_PRIORITY_CLASS,
                             NULL, NULL, &si, &pi);
    if (!ok) return false;
    CloseHandle(pi.hThread);
    CloseHandle(pi.hProcess);   // 不等待, 放手
    return true;
}

inline bool QPlotRunAsyncA(const char* exePath, const char* args)
{
    std::wstring wExe = QAcpToWide(exePath);
    std::wstring wArgs = QAcpToWide(args);
    return QPlotRunAsync(wExe.c_str(), wArgs.c_str());
}