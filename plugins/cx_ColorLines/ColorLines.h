/*
	ColorLines.h

	AE Plugin for Animation Composition - Color Line Extraction and Fill

	Main functionality:
	- Extract lines of specified color
	- Fill extracted lines with neighboring pixel colors
	- Color adjustments (brightness, contrast, saturation)
	- Sample blur within line mask
	- Multiple output modes
*/

#pragma once
#ifndef COLOR_LINES_H
#define COLOR_LINES_H

#include "AEConfig.h"
#include "entry.h"
#include "AEFX_SuiteHelper.h"
#include "PrSDKAESupport.h"
#include "AE_Effect.h"
#include "AE_EffectCB.h"
#include "AE_EffectCBSuites.h"
#include "AE_Macros.h"
#include "AEGP_SuiteHandler.h"
#include "String_Utils.h"
#include "Param_Utils.h"
#include "Smart_Utils.h"

#ifdef AE_OS_WIN
	#include <Windows.h>
#endif

#define DESCRIPTION	"\nColor Lines v1.0\r\nExtract and fill color lines for animation photography."

#define NAME			"cx_ColorLines"
#define	MAJOR_VERSION	1
#define	MINOR_VERSION	0
#define	BUG_VERSION		0
#define	STAGE_VERSION	PF_Stage_DEVELOP
#define	BUILD_VERSION	1

// Parameter IDs
enum {
	COLORLINES_INPUT = 0,

	// Color Selection Group
	COLORLINES_COLOR_GROUP_START,
	COLORLINES_TARGET_COLOR,
	COLORLINES_COLOR_TOLERANCE,
	COLORLINES_COLOR_GROUP_END,

	// Fill Settings Group
	COLORLINES_FILL_GROUP_START,
	COLORLINES_FILL_MODE,
	COLORLINES_SEARCH_RADIUS,
	COLORLINES_IGNORE_TRANSPARENT,
	COLORLINES_SAMPLE_BLUR,
	COLORLINES_FILL_GROUP_END,

	// Color Adjustments Group
	COLORLINES_ADJUST_GROUP_START,
	COLORLINES_BRIGHTNESS,
	COLORLINES_CONTRAST,
	COLORLINES_SATURATION,
	COLORLINES_ADJUST_GROUP_END,

	// Output Group
	COLORLINES_OUTPUT_GROUP_START,
	COLORLINES_OUTPUT_MODE,
	COLORLINES_OUTPUT_GROUP_END,

	COLORLINES_NUM_PARAMS
};

// Disk IDs for parameters (must be unique and persistent)
enum {
	COLOR_GROUP_START_DISK_ID = 1,
	TARGET_COLOR_DISK_ID,
	COLOR_TOLERANCE_DISK_ID,
	COLOR_GROUP_END_DISK_ID,

	FILL_GROUP_START_DISK_ID,
	FILL_MODE_DISK_ID,
	SEARCH_RADIUS_DISK_ID,
	IGNORE_TRANSPARENT_DISK_ID,
	SAMPLE_BLUR_DISK_ID,
	FILL_GROUP_END_DISK_ID,

	ADJUST_GROUP_START_DISK_ID,
	BRIGHTNESS_DISK_ID,
	CONTRAST_DISK_ID,
	SATURATION_DISK_ID,
	ADJUST_GROUP_END_DISK_ID,

	OUTPUT_GROUP_START_DISK_ID,
	OUTPUT_MODE_DISK_ID,
	OUTPUT_GROUP_END_DISK_ID
};

// Fill mode options
enum FillMode {
	FILL_MODE_NEAREST = 1,
	FILL_MODE_AVERAGE,
	FILL_MODE_WEIGHTED,
	FILL_MODE_NUM_MODES
};

// Output mode options
enum OutputMode {
	OUTPUT_MODE_FULL = 1,
	OUTPUT_MODE_LINE_ONLY,
	OUTPUT_MODE_BG_ONLY,
	OUTPUT_MODE_NUM_MODES
};

// Parameter defaults and ranges
#define TOLERANCE_MIN		0.0
#define TOLERANCE_MAX		100.0
#define TOLERANCE_DFLT		0.0

#define SEARCH_RADIUS_MIN	1
#define SEARCH_RADIUS_MAX	50
#define SEARCH_RADIUS_DFLT	5

#define SAMPLE_BLUR_MIN		0.0
#define SAMPLE_BLUR_MAX		100.0
#define SAMPLE_BLUR_DFLT	0.0

#define BRIGHTNESS_MIN		-100.0
#define BRIGHTNESS_MAX		100.0
#define BRIGHTNESS_DFLT		0.0

#define CONTRAST_MIN		-100.0
#define CONTRAST_MAX		100.0
#define CONTRAST_DFLT		0.0

#define SATURATION_MIN		-100.0
#define SATURATION_MAX		100.0
#define SATURATION_DFLT		0.0


extern "C" {

	DllExport
	PF_Err
	EffectMain (
		PF_Cmd			cmd,
		PF_InData		*in_data,
		PF_OutData		*out_data,
		PF_ParamDef		*params[],
		PF_LayerDef		*output,
		void			*extra);

}

// Plugin processing info structure
typedef struct ColorLinesInfo {
	// Color selection
	PF_Pixel		targetColor;
	PF_FpLong		tolerance;

	// Fill settings
	A_long			fillMode;
	A_long			searchRadius;
	PF_Boolean		ignoreTransparent;
	PF_FpLong		sampleBlur;

	// Color adjustments
	PF_FpLong		brightness;
	PF_FpLong		contrast;
	PF_FpLong		saturation;

	// Output
	A_long			outputMode;

	// Source image info for neighbor lookup
	PF_EffectWorld	*srcWorld;
	PF_InData		*in_data;

	// Extent offset for coordinate mapping
	A_long			x_offset;
	A_long			y_offset;

	// Line mask for multi-pass processing
	A_u_char		*lineMask;
	A_long			maskWidth;
	A_long			maskHeight;
	A_long			maskRowBytes;
} ColorLinesInfo, *ColorLinesInfoP, **ColorLinesInfoH;

// Pixel format structures for Premiere compatibility
typedef struct {
	A_u_char	blue, green, red, alpha;
} PF_Pixel_BGRA_8u;

typedef struct {
	PF_FpShort	blue, green, red, alpha;
} PF_Pixel_BGRA_32f;


#endif // COLOR_LINES_H
