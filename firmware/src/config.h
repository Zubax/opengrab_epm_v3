#ifndef CONFIG_H
#define CONFIG_H

#endif // CONFIG_H


#ifdef PRODROPPER
	#define TIME_OUT_MS 1000
	#define RATE_LIMIT_MS 3000
	#define VIN_MIN_MV 6700
	#define VIN_MAX_MV 15000
	#define PR_INDUCTANCE_PH 10000000
	#define BLAH 200
#else							//OpenGrab EPM V3
	#define TIME_OUT_MS 1000
	#define RATE_LIMIT_MS 2500
	#define VIN_MIN_MV 4300
	#define VIN_MAX_MV 6700
	#define PR_INDUCTANCE_PH 11000000
	#define BLAH 450
#endif
