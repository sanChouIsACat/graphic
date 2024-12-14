//#include "monitorHelper.hpp"
//#include <windows.h>
// using namespace std;
// using namespace g_os;
//
// BOOL CALLBACK MonitorEnumProc(HMONITOR hMonitor, HDC hdcMonitor, LPRECT
// lprcMonitor, LPARAM dwData) {
//    std::vector<MonitorInfo>* monitors =
//    reinterpret_cast<std::vector<MonitorInfo>*>(dwData); MonitorInfo info;
//    info.x = lprcMonitor->left;
//    info.y = lprcMonitor->top;
//    info.width = lprcMonitor->right - lprcMonitor->left;
//    info.height = lprcMonitor->bottom - lprcMonitor->top;
//    monitors->push_back(info);
//    return TRUE;
//}
// const std::vector<MonitorInfo>& g_os::MonitorHelper::getMonitors()
//{
//    return monitors;
//}
//
// void g_os::MonitorHelper::getMonitorInfos()
//{
//    EnumDisplayMonitors(NULL, NULL, MonitorEnumProc,
//    reinterpret_cast<LPARAM>(&monitors));
//}
