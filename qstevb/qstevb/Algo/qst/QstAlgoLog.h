
#ifndef _QSTALGOLOG_H
#define _QSTALGOLOG_H

//#ifdef TAG
#undef LOG_TAG
#define LOG_TAG			"qst_algo"
//#endif

#define QST_ALGO_DEBUG

//#define USE_ANDROID_STUDIO
//#define USE_ANDROID_PLATFORM
//#define USE_MCU_PLATFORM
#define USE_VISUAL_STUDIO

#if defined(QST_ALGO_DEBUG)
#if defined(USE_ANDROID_STUDIO)
	#include <android/log.h>
	#define LOGD(...)		__android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
	#define LOGI(...)		__android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
	#define LOGW(...)		__android_log_print(ANDROID_LOG_WARN, LOG_TAG, __VA_ARGS__)
	#define LOGE(...)		__android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
	#define LOGF(...)		__android_log_print(ANDROID_LOG_FATAL, LOG_TAG, __VA_ARGS__)
#elif defined(USE_ANDROID_PLATFORM)
	#include <log/log.h>
	//#include <cutils/log.h>
	#define LOGD(...)		ALOGD(__VA_ARGS__)
	#define LOGI(...)		ALOGI(__VA_ARGS__)
	#define LOGW(...)		ALOGW(__VA_ARGS__)
	#define LOGE(...)		ALOGE(__VA_ARGS__)
	#define LOGF(...)		ALOGF(__VA_ARGS__)
#elif defined(USE_MCU_PLATFORM)
	#define LOGD(...)		printf(__VA_ARGS__); printf("\n")
	#define LOGI(...)		printf(__VA_ARGS__)
	#define LOGW(...)		printf(__VA_ARGS__)
	#define LOGE(...)		printf(__VA_ARGS__)
	#define LOGF(...)		printf(__VA_ARGS__)
#elif defined(USE_VISUAL_STUDIO)
	#include "conio.h"
	#define LOGD(...)		_cprintf(__VA_ARGS__); _cprintf("\n")
	#define LOGI(...)		_cprintf(__VA_ARGS__)
	#define LOGW(...)		_cprintf(__VA_ARGS__)
	#define LOGE(...)		_cprintf(__VA_ARGS__)
	#define LOGF(...)		_cprintf(__VA_ARGS__)
#else
	#define LOGD(...)
	#define LOGI(...)
	#define LOGW(...)
	#define LOGE(...)
	#define LOGF(...)
#endif

#else
#define LOGD(...)
#define LOGI(...)
#define LOGW(...)
#define LOGE(...)
#define LOGF(...)
#endif

#endif // _QSTALGOLOG_H

