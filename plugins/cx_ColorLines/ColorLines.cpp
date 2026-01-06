/*
	ColorLines.cpp

	AE Plugin for Animation Composition - Color Line Extraction and Fill
	Supports 8-bit, 16-bit, and 32-bit float color processing

	Optimized for performance:
	- Precomputed lookup tables for distance weights
	- Squared distance comparisons (avoid sqrt)
	- Cached row pointers for faster pixel access
	- Precomputed color adjustment factors
*/

#include "ColorLines.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

// ============================================================================
// Precomputed Tables and Constants
// ============================================================================

// Maximum search radius for weight table
#define MAX_WEIGHT_TABLE_RADIUS 50
#define WEIGHT_TABLE_SIZE ((MAX_WEIGHT_TABLE_RADIUS * 2 + 1) * (MAX_WEIGHT_TABLE_RADIUS * 2 + 1))

// Precomputed inverse distance weights for weighted average mode
// Index: (dy + radius) * (radius * 2 + 1) + (dx + radius)
static PF_FpLong g_invDistWeights[WEIGHT_TABLE_SIZE];
static PF_FpLong g_gaussianWeights[WEIGHT_TABLE_SIZE];
static A_long g_currentWeightRadius = 0;
static A_long g_currentBlurRadius = 0;

// Precompute inverse distance weight table
static void PrecomputeInvDistWeights(A_long radius) {
	if (radius == g_currentWeightRadius) return;
	if (radius > MAX_WEIGHT_TABLE_RADIUS) radius = MAX_WEIGHT_TABLE_RADIUS;

	A_long size = radius * 2 + 1;
	for (A_long dy = -radius; dy <= radius; dy++) {
		for (A_long dx = -radius; dx <= radius; dx++) {
			A_long idx = (dy + radius) * size + (dx + radius);
			if (dx == 0 && dy == 0) {
				g_invDistWeights[idx] = 0.0;
			} else {
				PF_FpLong dist = sqrt((PF_FpLong)(dx * dx + dy * dy));
				g_invDistWeights[idx] = 1.0 / (dist + 0.1);
			}
		}
	}
	g_currentWeightRadius = radius;
}

// Precompute gaussian weight table for blur
static void PrecomputeGaussianWeights(A_long blurRadius) {
	if (blurRadius == g_currentBlurRadius) return;
	if (blurRadius > MAX_WEIGHT_TABLE_RADIUS) blurRadius = MAX_WEIGHT_TABLE_RADIUS;

	A_long size = blurRadius * 2 + 1;
	PF_FpLong sigma2 = 2.0 * blurRadius * blurRadius;

	for (A_long dy = -blurRadius; dy <= blurRadius; dy++) {
		for (A_long dx = -blurRadius; dx <= blurRadius; dx++) {
			A_long idx = (dy + blurRadius) * size + (dx + blurRadius);
			A_long distSq = dx * dx + dy * dy;
			g_gaussianWeights[idx] = exp(-(PF_FpLong)distSq / sigma2);
		}
	}
	g_currentBlurRadius = blurRadius;
}

// ============================================================================
// Optimized Utility Functions
// ============================================================================

static inline A_u_char ClampByte(PF_FpLong value) {
	return (A_u_char)(value < 0 ? 0 : (value > 255 ? 255 : value));
}

static inline A_u_short Clamp16(PF_FpLong value) {
	return (A_u_short)(value < 0 ? 0 : (value > PF_MAX_CHAN16 ? PF_MAX_CHAN16 : value));
}

static inline PF_FpLong Clamp01(PF_FpLong value) {
	return value < 0.0 ? 0.0 : (value > 1.0 ? 1.0 : value);
}

static void UnionLRect(const PF_LRect *src, PF_LRect *dst) {
	if (src->left < dst->left) dst->left = src->left;
	if (src->top < dst->top) dst->top = src->top;
	if (src->right > dst->right) dst->right = src->right;
	if (src->bottom > dst->bottom) dst->bottom = src->bottom;
}

// Fast mask access - no bounds check (caller ensures bounds)
static inline A_u_char GetMaskAtFast(ColorLinesInfo *info, A_long x, A_long y) {
	return info->lineMask[y * info->maskRowBytes + x];
}

static inline void SetMaskAtFast(ColorLinesInfo *info, A_long x, A_long y, A_u_char value) {
	info->lineMask[y * info->maskRowBytes + x] = value;
}

// Safe mask access with bounds check
static inline A_u_char GetMaskAt(ColorLinesInfo *info, A_long x, A_long y) {
	if (x < 0 || x >= info->maskWidth || y < 0 || y >= info->maskHeight) return 0;
	return info->lineMask[y * info->maskRowBytes + x];
}

// ============================================================================
// Optimized Pixel Access - Direct pointer arithmetic
// ============================================================================

// Get row pointer once, then access pixels directly
static inline PF_Pixel8* GetRow8(PF_EffectWorld *world, A_long y) {
	return (PF_Pixel8*)((char*)world->data + y * world->rowbytes);
}

static inline PF_Pixel16* GetRow16(PF_EffectWorld *world, A_long y) {
	return (PF_Pixel16*)((char*)world->data + y * world->rowbytes);
}

static inline PF_PixelFloat* GetRowFloat(PF_EffectWorld *world, A_long y) {
	return (PF_PixelFloat*)((char*)world->data + y * world->rowbytes);
}

// ============================================================================
// RGB <-> HSL Conversion (optimized)
// ============================================================================

static inline PF_FpLong HueToRGB(PF_FpLong p, PF_FpLong q, PF_FpLong t) {
	if (t < 0.0) t += 1.0;
	else if (t > 1.0) t -= 1.0;

	if (t < 0.166666667) return p + (q - p) * 6.0 * t;
	if (t < 0.5) return q;
	if (t < 0.666666667) return p + (q - p) * (0.666666667 - t) * 6.0;
	return p;
}

static void RGBtoHSL(PF_FpLong r, PF_FpLong g, PF_FpLong b, PF_FpLong *h, PF_FpLong *s, PF_FpLong *l) {
	PF_FpLong maxVal = r > g ? (r > b ? r : b) : (g > b ? g : b);
	PF_FpLong minVal = r < g ? (r < b ? r : b) : (g < b ? g : b);
	PF_FpLong delta = maxVal - minVal;

	*l = (maxVal + minVal) * 0.5;

	if (delta < 0.00001) {
		*h = 0.0;
		*s = 0.0;
	} else {
		*s = (*l > 0.5) ? delta / (2.0 - maxVal - minVal) : delta / (maxVal + minVal);
		if (maxVal == r) {
			*h = (g - b) / delta + (g < b ? 6.0 : 0.0);
		} else if (maxVal == g) {
			*h = (b - r) / delta + 2.0;
		} else {
			*h = (r - g) / delta + 4.0;
		}
		*h *= 0.166666667;
	}
}

static void HSLtoRGB(PF_FpLong h, PF_FpLong s, PF_FpLong l, PF_FpLong *r, PF_FpLong *g, PF_FpLong *b) {
	if (s < 0.00001) {
		*r = *g = *b = l;
	} else {
		PF_FpLong q = l < 0.5 ? l * (1.0 + s) : l + s - l * s;
		PF_FpLong p = 2.0 * l - q;
		*r = HueToRGB(p, q, h + 0.333333333);
		*g = HueToRGB(p, q, h);
		*b = HueToRGB(p, q, h - 0.333333333);
	}
}

// ============================================================================
// Optimized Color Matching - Use squared distance
// ============================================================================

// Precompute tolerance squared: maxDistance = tolerance * 4.4167, store maxDistanceSq
static inline PF_Boolean IsTargetColor8Fast(PF_Pixel8 *pixel, A_long targetR, A_long targetG, A_long targetB, PF_FpLong toleranceSq) {
	A_long dr = (A_long)pixel->red - targetR;
	A_long dg = (A_long)pixel->green - targetG;
	A_long db = (A_long)pixel->blue - targetB;
	A_long distSq = dr * dr + dg * dg + db * db;
	return ((PF_FpLong)distSq <= toleranceSq);
}

static inline PF_Boolean IsTargetColor16Fast(PF_Pixel16 *pixel, PF_FpLong targetR, PF_FpLong targetG, PF_FpLong targetB, PF_FpLong toleranceSq) {
	PF_FpLong dr = (PF_FpLong)pixel->red - targetR;
	PF_FpLong dg = (PF_FpLong)pixel->green - targetG;
	PF_FpLong db = (PF_FpLong)pixel->blue - targetB;
	PF_FpLong distSq = dr * dr + dg * dg + db * db;
	return (distSq <= toleranceSq);
}

static inline PF_Boolean IsTargetColorFloatFast(PF_PixelFloat *pixel, PF_FpLong targetR, PF_FpLong targetG, PF_FpLong targetB, PF_FpLong toleranceSq) {
	PF_FpLong dr = pixel->red - targetR;
	PF_FpLong dg = pixel->green - targetG;
	PF_FpLong db = pixel->blue - targetB;
	PF_FpLong distSq = dr * dr + dg * dg + db * db;
	return (distSq <= toleranceSq);
}

// Legacy functions for compatibility
PF_Boolean IsTargetColor8(PF_Pixel8 *pixel, PF_Pixel *targetColor, PF_FpLong tolerance) {
	PF_FpLong maxDist = tolerance * 4.4167;
	return IsTargetColor8Fast(pixel, targetColor->red, targetColor->green, targetColor->blue, maxDist * maxDist);
}

PF_Boolean IsTargetColor16(PF_Pixel16 *pixel, PF_Pixel *targetColor, PF_FpLong tolerance) {
	const PF_FpLong scale8to16 = PF_MAX_CHAN16 / 255.0;
	PF_FpLong targetR = targetColor->red * scale8to16;
	PF_FpLong targetG = targetColor->green * scale8to16;
	PF_FpLong targetB = targetColor->blue * scale8to16;
	PF_FpLong maxDist = tolerance * 4.4167 * scale8to16;
	return IsTargetColor16Fast(pixel, targetR, targetG, targetB, maxDist * maxDist);
}

PF_Boolean IsTargetColorFloat(PF_PixelFloat *pixel, PF_Pixel *targetColor, PF_FpLong tolerance) {
	const PF_FpLong scale8toFloat = 1.0 / 255.0;
	PF_FpLong targetR = targetColor->red * scale8toFloat;
	PF_FpLong targetG = targetColor->green * scale8toFloat;
	PF_FpLong targetB = targetColor->blue * scale8toFloat;
	PF_FpLong maxDist = tolerance * 4.4167 * scale8toFloat;
	return IsTargetColorFloatFast(pixel, targetR, targetG, targetB, maxDist * maxDist);
}

// ============================================================================
// Precomputed Color Adjustment Factors
// ============================================================================

typedef struct {
	PF_Boolean needsAdjustment;
	PF_Boolean needsBrightness;
	PF_Boolean needsContrast;
	PF_Boolean needsSaturation;
	PF_FpLong brightnessFactor;
	PF_FpLong contrastFactor;
	PF_FpLong saturationFactor;
} ColorAdjustParams;

static void InitColorAdjustParams(ColorAdjustParams *adj, ColorLinesInfo *info) {
	adj->needsBrightness = (info->brightness != 0.0);
	adj->needsContrast = (info->contrast != 0.0);
	adj->needsSaturation = (info->saturation != 0.0);
	adj->needsAdjustment = adj->needsBrightness || adj->needsContrast || adj->needsSaturation;

	if (adj->needsBrightness) {
		adj->brightnessFactor = info->brightness / 100.0;
	}
	if (adj->needsContrast) {
		adj->contrastFactor = (100.0 + info->contrast) / 100.0;
		adj->contrastFactor *= adj->contrastFactor;
	}
	if (adj->needsSaturation) {
		adj->saturationFactor = (100.0 + info->saturation) / 100.0;
	}
}

static inline void ApplyColorAdjustments8Fast(PF_Pixel8 *pixel, const ColorAdjustParams *adj) {
	if (!adj->needsAdjustment) return;

	PF_FpLong r = pixel->red * 0.00392156863;  // / 255.0
	PF_FpLong g = pixel->green * 0.00392156863;
	PF_FpLong b = pixel->blue * 0.00392156863;

	if (adj->needsBrightness) {
		r = Clamp01(r + adj->brightnessFactor);
		g = Clamp01(g + adj->brightnessFactor);
		b = Clamp01(b + adj->brightnessFactor);
	}

	if (adj->needsContrast) {
		r = Clamp01(0.5 + (r - 0.5) * adj->contrastFactor);
		g = Clamp01(0.5 + (g - 0.5) * adj->contrastFactor);
		b = Clamp01(0.5 + (b - 0.5) * adj->contrastFactor);
	}

	if (adj->needsSaturation) {
		PF_FpLong h, s, l;
		RGBtoHSL(r, g, b, &h, &s, &l);
		s = Clamp01(s * adj->saturationFactor);
		HSLtoRGB(h, s, l, &r, &g, &b);
	}

	pixel->red = ClampByte(r * 255.0);
	pixel->green = ClampByte(g * 255.0);
	pixel->blue = ClampByte(b * 255.0);
}

static inline void ApplyColorAdjustments16Fast(PF_Pixel16 *pixel, const ColorAdjustParams *adj) {
	if (!adj->needsAdjustment) return;

	const PF_FpLong invMax = 1.0 / PF_MAX_CHAN16;
	PF_FpLong r = pixel->red * invMax;
	PF_FpLong g = pixel->green * invMax;
	PF_FpLong b = pixel->blue * invMax;

	if (adj->needsBrightness) {
		r = Clamp01(r + adj->brightnessFactor);
		g = Clamp01(g + adj->brightnessFactor);
		b = Clamp01(b + adj->brightnessFactor);
	}

	if (adj->needsContrast) {
		r = Clamp01(0.5 + (r - 0.5) * adj->contrastFactor);
		g = Clamp01(0.5 + (g - 0.5) * adj->contrastFactor);
		b = Clamp01(0.5 + (b - 0.5) * adj->contrastFactor);
	}

	if (adj->needsSaturation) {
		PF_FpLong h, s, l;
		RGBtoHSL(r, g, b, &h, &s, &l);
		s = Clamp01(s * adj->saturationFactor);
		HSLtoRGB(h, s, l, &r, &g, &b);
	}

	pixel->red = Clamp16(r * PF_MAX_CHAN16);
	pixel->green = Clamp16(g * PF_MAX_CHAN16);
	pixel->blue = Clamp16(b * PF_MAX_CHAN16);
}

static inline void ApplyColorAdjustmentsFloatFast(PF_PixelFloat *pixel, const ColorAdjustParams *adj) {
	if (!adj->needsAdjustment) return;

	PF_FpLong r = pixel->red;
	PF_FpLong g = pixel->green;
	PF_FpLong b = pixel->blue;

	if (adj->needsBrightness) {
		r += adj->brightnessFactor;
		g += adj->brightnessFactor;
		b += adj->brightnessFactor;
	}

	if (adj->needsContrast) {
		r = 0.5 + (r - 0.5) * adj->contrastFactor;
		g = 0.5 + (g - 0.5) * adj->contrastFactor;
		b = 0.5 + (b - 0.5) * adj->contrastFactor;
	}

	if (adj->needsSaturation) {
		PF_FpLong h, s, l;
		RGBtoHSL(Clamp01(r), Clamp01(g), Clamp01(b), &h, &s, &l);
		s = Clamp01(s * adj->saturationFactor);
		HSLtoRGB(h, s, l, &r, &g, &b);
	}

	pixel->red = (PF_FpShort)r;
	pixel->green = (PF_FpShort)g;
	pixel->blue = (PF_FpShort)b;
}

// Legacy wrappers
void ApplyColorAdjustments8(PF_Pixel8 *pixel, ColorLinesInfo *info) {
	ColorAdjustParams adj;
	InitColorAdjustParams(&adj, info);
	ApplyColorAdjustments8Fast(pixel, &adj);
}

void ApplyColorAdjustments16(PF_Pixel16 *pixel, ColorLinesInfo *info) {
	ColorAdjustParams adj;
	InitColorAdjustParams(&adj, info);
	ApplyColorAdjustments16Fast(pixel, &adj);
}

void ApplyColorAdjustmentsFloat(PF_PixelFloat *pixel, ColorLinesInfo *info) {
	ColorAdjustParams adj;
	InitColorAdjustParams(&adj, info);
	ApplyColorAdjustmentsFloatFast(pixel, &adj);
}

// ============================================================================
// Optimized Fill Functions
// ============================================================================

static void FillLinePixel8(ColorLinesInfo *info, A_long x, A_long y, PF_Pixel8 *inP, PF_Pixel8 *outP,
                           A_long targetR, A_long targetG, A_long targetB, PF_FpLong toleranceSq,
                           const ColorAdjustParams *adj) {
	A_long radius = info->searchRadius;
	A_long width = info->srcWorld->width;
	A_long height = info->srcWorld->height;
	A_long weightSize = radius * 2 + 1;

	if (info->fillMode == FILL_MODE_NEAREST) {
		// Find nearest non-target pixel
		A_long nearestDistSq = 999999;
		PF_Pixel8 *nearestPixel = NULL;

		// Search in expanding rings for early termination
		for (A_long ring = 1; ring <= radius && nearestDistSq > 1; ring++) {
			A_long ringSq = ring * ring;
			if (ringSq >= nearestDistSq) break;  // Can't find closer

			for (A_long dy = -ring; dy <= ring; dy++) {
				A_long ny = y + dy;
				if (ny < 0 || ny >= height) continue;

				PF_Pixel8 *rowPtr = GetRow8(info->srcWorld, ny);

				for (A_long dx = -ring; dx <= ring; dx++) {
					// Only process ring boundary
					if (dy != -ring && dy != ring && dx != -ring && dx != ring) continue;

					A_long nx = x + dx;
					if (nx < 0 || nx >= width) continue;

					PF_Pixel8 *neighbor = rowPtr + nx;
					if (info->ignoreTransparent && neighbor->alpha < 255) continue;
					if (IsTargetColor8Fast(neighbor, targetR, targetG, targetB, toleranceSq)) continue;

					A_long distSq = dx * dx + dy * dy;
					if (distSq < nearestDistSq) {
						nearestDistSq = distSq;
						nearestPixel = neighbor;
						if (distSq == 1) goto found_nearest;  // Can't get closer
					}
				}
			}
		}
		found_nearest:

		if (nearestPixel) {
			*outP = *nearestPixel;
		} else {
			*outP = *inP;
		}
	} else {
		// Average or Weighted mode
		PF_FpLong totalWeight = 0;
		PF_FpLong sumR = 0, sumG = 0, sumB = 0, sumA = 0;
		PF_Boolean isAverage = (info->fillMode == FILL_MODE_AVERAGE);

		for (A_long dy = -radius; dy <= radius; dy++) {
			A_long ny = y + dy;
			if (ny < 0 || ny >= height) continue;

			PF_Pixel8 *rowPtr = GetRow8(info->srcWorld, ny);
			A_long weightRowOffset = (dy + radius) * weightSize;

			for (A_long dx = -radius; dx <= radius; dx++) {
				if (dx == 0 && dy == 0) continue;

				A_long nx = x + dx;
				if (nx < 0 || nx >= width) continue;

				PF_Pixel8 *neighbor = rowPtr + nx;
				if (info->ignoreTransparent && neighbor->alpha < 255) continue;
				if (IsTargetColor8Fast(neighbor, targetR, targetG, targetB, toleranceSq)) continue;

				PF_FpLong weight = isAverage ? 1.0 : g_invDistWeights[weightRowOffset + dx + radius];
				sumR += neighbor->red * weight;
				sumG += neighbor->green * weight;
				sumB += neighbor->blue * weight;
				sumA += neighbor->alpha * weight;
				totalWeight += weight;
			}
		}

		if (totalWeight > 0) {
			PF_FpLong invWeight = 1.0 / totalWeight;
			outP->red = ClampByte(sumR * invWeight);
			outP->green = ClampByte(sumG * invWeight);
			outP->blue = ClampByte(sumB * invWeight);
			outP->alpha = ClampByte(sumA * invWeight);
		} else {
			*outP = *inP;
		}
	}
	ApplyColorAdjustments8Fast(outP, adj);
}

static void FillLinePixel16(ColorLinesInfo *info, A_long x, A_long y, PF_Pixel16 *inP, PF_Pixel16 *outP,
                            PF_FpLong targetR, PF_FpLong targetG, PF_FpLong targetB, PF_FpLong toleranceSq,
                            const ColorAdjustParams *adj) {
	A_long radius = info->searchRadius;
	A_long width = info->srcWorld->width;
	A_long height = info->srcWorld->height;
	A_long weightSize = radius * 2 + 1;

	if (info->fillMode == FILL_MODE_NEAREST) {
		A_long nearestDistSq = 999999;
		PF_Pixel16 *nearestPixel = NULL;

		for (A_long ring = 1; ring <= radius && nearestDistSq > 1; ring++) {
			A_long ringSq = ring * ring;
			if (ringSq >= nearestDistSq) break;

			for (A_long dy = -ring; dy <= ring; dy++) {
				A_long ny = y + dy;
				if (ny < 0 || ny >= height) continue;

				PF_Pixel16 *rowPtr = GetRow16(info->srcWorld, ny);

				for (A_long dx = -ring; dx <= ring; dx++) {
					if (dy != -ring && dy != ring && dx != -ring && dx != ring) continue;

					A_long nx = x + dx;
					if (nx < 0 || nx >= width) continue;

					PF_Pixel16 *neighbor = rowPtr + nx;
					if (info->ignoreTransparent && neighbor->alpha < PF_MAX_CHAN16) continue;
					if (IsTargetColor16Fast(neighbor, targetR, targetG, targetB, toleranceSq)) continue;

					A_long distSq = dx * dx + dy * dy;
					if (distSq < nearestDistSq) {
						nearestDistSq = distSq;
						nearestPixel = neighbor;
						if (distSq == 1) goto found_nearest16;
					}
				}
			}
		}
		found_nearest16:

		if (nearestPixel) {
			*outP = *nearestPixel;
		} else {
			*outP = *inP;
		}
	} else {
		PF_FpLong totalWeight = 0;
		PF_FpLong sumR = 0, sumG = 0, sumB = 0, sumA = 0;
		PF_Boolean isAverage = (info->fillMode == FILL_MODE_AVERAGE);

		for (A_long dy = -radius; dy <= radius; dy++) {
			A_long ny = y + dy;
			if (ny < 0 || ny >= height) continue;

			PF_Pixel16 *rowPtr = GetRow16(info->srcWorld, ny);
			A_long weightRowOffset = (dy + radius) * weightSize;

			for (A_long dx = -radius; dx <= radius; dx++) {
				if (dx == 0 && dy == 0) continue;

				A_long nx = x + dx;
				if (nx < 0 || nx >= width) continue;

				PF_Pixel16 *neighbor = rowPtr + nx;
				if (info->ignoreTransparent && neighbor->alpha < PF_MAX_CHAN16) continue;
				if (IsTargetColor16Fast(neighbor, targetR, targetG, targetB, toleranceSq)) continue;

				PF_FpLong weight = isAverage ? 1.0 : g_invDistWeights[weightRowOffset + dx + radius];
				sumR += neighbor->red * weight;
				sumG += neighbor->green * weight;
				sumB += neighbor->blue * weight;
				sumA += neighbor->alpha * weight;
				totalWeight += weight;
			}
		}

		if (totalWeight > 0) {
			PF_FpLong invWeight = 1.0 / totalWeight;
			outP->red = Clamp16(sumR * invWeight);
			outP->green = Clamp16(sumG * invWeight);
			outP->blue = Clamp16(sumB * invWeight);
			outP->alpha = Clamp16(sumA * invWeight);
		} else {
			*outP = *inP;
		}
	}
	ApplyColorAdjustments16Fast(outP, adj);
}

static void FillLinePixelFloat(ColorLinesInfo *info, A_long x, A_long y, PF_PixelFloat *inP, PF_PixelFloat *outP,
                               PF_FpLong targetR, PF_FpLong targetG, PF_FpLong targetB, PF_FpLong toleranceSq,
                               const ColorAdjustParams *adj) {
	A_long radius = info->searchRadius;
	A_long width = info->srcWorld->width;
	A_long height = info->srcWorld->height;
	A_long weightSize = radius * 2 + 1;

	if (info->fillMode == FILL_MODE_NEAREST) {
		A_long nearestDistSq = 999999;
		PF_PixelFloat *nearestPixel = NULL;

		for (A_long ring = 1; ring <= radius && nearestDistSq > 1; ring++) {
			A_long ringSq = ring * ring;
			if (ringSq >= nearestDistSq) break;

			for (A_long dy = -ring; dy <= ring; dy++) {
				A_long ny = y + dy;
				if (ny < 0 || ny >= height) continue;

				PF_PixelFloat *rowPtr = GetRowFloat(info->srcWorld, ny);

				for (A_long dx = -ring; dx <= ring; dx++) {
					if (dy != -ring && dy != ring && dx != -ring && dx != ring) continue;

					A_long nx = x + dx;
					if (nx < 0 || nx >= width) continue;

					PF_PixelFloat *neighbor = rowPtr + nx;
					if (info->ignoreTransparent && neighbor->alpha < 1.0f) continue;
					if (IsTargetColorFloatFast(neighbor, targetR, targetG, targetB, toleranceSq)) continue;

					A_long distSq = dx * dx + dy * dy;
					if (distSq < nearestDistSq) {
						nearestDistSq = distSq;
						nearestPixel = neighbor;
						if (distSq == 1) goto found_nearestF;
					}
				}
			}
		}
		found_nearestF:

		if (nearestPixel) {
			*outP = *nearestPixel;
		} else {
			*outP = *inP;
		}
	} else {
		PF_FpLong totalWeight = 0;
		PF_FpLong sumR = 0, sumG = 0, sumB = 0, sumA = 0;
		PF_Boolean isAverage = (info->fillMode == FILL_MODE_AVERAGE);

		for (A_long dy = -radius; dy <= radius; dy++) {
			A_long ny = y + dy;
			if (ny < 0 || ny >= height) continue;

			PF_PixelFloat *rowPtr = GetRowFloat(info->srcWorld, ny);
			A_long weightRowOffset = (dy + radius) * weightSize;

			for (A_long dx = -radius; dx <= radius; dx++) {
				if (dx == 0 && dy == 0) continue;

				A_long nx = x + dx;
				if (nx < 0 || nx >= width) continue;

				PF_PixelFloat *neighbor = rowPtr + nx;
				if (info->ignoreTransparent && neighbor->alpha < 1.0f) continue;
				if (IsTargetColorFloatFast(neighbor, targetR, targetG, targetB, toleranceSq)) continue;

				PF_FpLong weight = isAverage ? 1.0 : g_invDistWeights[weightRowOffset + dx + radius];
				sumR += neighbor->red * weight;
				sumG += neighbor->green * weight;
				sumB += neighbor->blue * weight;
				sumA += neighbor->alpha * weight;
				totalWeight += weight;
			}
		}

		if (totalWeight > 0) {
			PF_FpLong invWeight = 1.0 / totalWeight;
			outP->red = (PF_FpShort)(sumR * invWeight);
			outP->green = (PF_FpShort)(sumG * invWeight);
			outP->blue = (PF_FpShort)(sumB * invWeight);
			outP->alpha = (PF_FpShort)(sumA * invWeight);
		} else {
			*outP = *inP;
		}
	}
	ApplyColorAdjustmentsFloatFast(outP, adj);
}

// ============================================================================
// Extended Info for Optimized Processing
// ============================================================================

typedef struct {
	ColorLinesInfo *info;
	// Precomputed target color (converted for bit depth)
	A_long targetR8, targetG8, targetB8;
	PF_FpLong targetR16, targetG16, targetB16;
	PF_FpLong targetRF, targetGF, targetBF;
	PF_FpLong toleranceSq8, toleranceSq16, toleranceSqF;
	ColorAdjustParams colorAdj;
	A_long edgeMargin;
	A_long width, height;
} ProcessingContext;

static void InitProcessingContext(ProcessingContext *ctx, ColorLinesInfo *info) {
	ctx->info = info;
	ctx->edgeMargin = info->searchRadius;
	ctx->width = info->srcWorld->width;
	ctx->height = info->srcWorld->height;

	// 8-bit targets
	ctx->targetR8 = info->targetColor.red;
	ctx->targetG8 = info->targetColor.green;
	ctx->targetB8 = info->targetColor.blue;
	PF_FpLong maxDist8 = info->tolerance * 4.4167;
	ctx->toleranceSq8 = maxDist8 * maxDist8;

	// 16-bit targets (8-bit 0-255 -> 16-bit 0-32768)
	// Correct conversion factor: PF_MAX_CHAN16 / 255.0 = 32768 / 255.0 ≈ 128.502
	const PF_FpLong scale8to16 = PF_MAX_CHAN16 / 255.0;
	ctx->targetR16 = info->targetColor.red * scale8to16;
	ctx->targetG16 = info->targetColor.green * scale8to16;
	ctx->targetB16 = info->targetColor.blue * scale8to16;
	PF_FpLong maxDist16 = info->tolerance * 4.4167 * scale8to16;
	ctx->toleranceSq16 = maxDist16 * maxDist16;

	// Float targets (8-bit 0-255 -> float 0.0-1.0)
	const PF_FpLong scale8toFloat = 1.0 / 255.0;
	ctx->targetRF = info->targetColor.red * scale8toFloat;
	ctx->targetGF = info->targetColor.green * scale8toFloat;
	ctx->targetBF = info->targetColor.blue * scale8toFloat;
	PF_FpLong maxDistF = info->tolerance * 4.4167 * scale8toFloat;
	ctx->toleranceSqF = maxDistF * maxDistF;

	// Color adjustments
	InitColorAdjustParams(&ctx->colorAdj, info);

	// Precompute weight tables if needed
	if (info->fillMode == FILL_MODE_WEIGHTED) {
		PrecomputeInvDistWeights(info->searchRadius);
	}
}

// ============================================================================
// Optimized Pixel Processing Callbacks
// ============================================================================

static PF_Err FillAndMask8_Optimized(void *refcon, A_long xL, A_long yL, PF_Pixel8 *inP, PF_Pixel8 *outP) {
	ProcessingContext *ctx = (ProcessingContext*)refcon;
	ColorLinesInfo *info = ctx->info;

	// Skip edge pixels
	if (xL < ctx->edgeMargin || yL < ctx->edgeMargin ||
		xL >= ctx->width - ctx->edgeMargin ||
		yL >= ctx->height - ctx->edgeMargin) {
		*outP = *inP;
		SetMaskAtFast(info, xL, yL, 0);
		return PF_Err_NONE;
	}

	PF_Boolean isLine = IsTargetColor8Fast(inP, ctx->targetR8, ctx->targetG8, ctx->targetB8, ctx->toleranceSq8);
	SetMaskAtFast(info, xL, yL, isLine ? 255 : 0);

	switch (info->outputMode) {
		case OUTPUT_MODE_FULL:
			if (isLine) {
				FillLinePixel8(info, xL, yL, inP, outP, ctx->targetR8, ctx->targetG8, ctx->targetB8, ctx->toleranceSq8, &ctx->colorAdj);
			} else {
				*outP = *inP;
			}
			break;
		case OUTPUT_MODE_LINE_ONLY:
			if (isLine) {
				FillLinePixel8(info, xL, yL, inP, outP, ctx->targetR8, ctx->targetG8, ctx->targetB8, ctx->toleranceSq8, &ctx->colorAdj);
				outP->alpha = 255;
			} else {
				outP->alpha = 0;
				outP->red = 0;
				outP->green = 0;
				outP->blue = 0;
			}
			break;
		case OUTPUT_MODE_BG_ONLY:
			if (isLine) {
				outP->alpha = 0;
				outP->red = 0;
				outP->green = 0;
				outP->blue = 0;
			} else {
				*outP = *inP;
			}
			break;
		default:
			*outP = *inP;
			break;
	}
	return PF_Err_NONE;
}

static PF_Err FillAndMask16_Optimized(void *refcon, A_long xL, A_long yL, PF_Pixel16 *inP, PF_Pixel16 *outP) {
	ProcessingContext *ctx = (ProcessingContext*)refcon;
	ColorLinesInfo *info = ctx->info;

	if (xL < ctx->edgeMargin || yL < ctx->edgeMargin ||
		xL >= ctx->width - ctx->edgeMargin ||
		yL >= ctx->height - ctx->edgeMargin) {
		*outP = *inP;
		SetMaskAtFast(info, xL, yL, 0);
		return PF_Err_NONE;
	}

	PF_Boolean isLine = IsTargetColor16Fast(inP, ctx->targetR16, ctx->targetG16, ctx->targetB16, ctx->toleranceSq16);
	SetMaskAtFast(info, xL, yL, isLine ? 255 : 0);

	switch (info->outputMode) {
		case OUTPUT_MODE_FULL:
			if (isLine) {
				FillLinePixel16(info, xL, yL, inP, outP, ctx->targetR16, ctx->targetG16, ctx->targetB16, ctx->toleranceSq16, &ctx->colorAdj);
			} else {
				*outP = *inP;
			}
			break;
		case OUTPUT_MODE_LINE_ONLY:
			if (isLine) {
				FillLinePixel16(info, xL, yL, inP, outP, ctx->targetR16, ctx->targetG16, ctx->targetB16, ctx->toleranceSq16, &ctx->colorAdj);
				outP->alpha = PF_MAX_CHAN16;
			} else {
				outP->alpha = 0;
				outP->red = 0;
				outP->green = 0;
				outP->blue = 0;
			}
			break;
		case OUTPUT_MODE_BG_ONLY:
			if (isLine) {
				outP->alpha = 0;
				outP->red = 0;
				outP->green = 0;
				outP->blue = 0;
			} else {
				*outP = *inP;
			}
			break;
		default:
			*outP = *inP;
			break;
	}
	return PF_Err_NONE;
}

static PF_Err FillAndMaskFloat_Optimized(void *refcon, A_long xL, A_long yL, PF_PixelFloat *inP, PF_PixelFloat *outP) {
	ProcessingContext *ctx = (ProcessingContext*)refcon;
	ColorLinesInfo *info = ctx->info;

	if (xL < ctx->edgeMargin || yL < ctx->edgeMargin ||
		xL >= ctx->width - ctx->edgeMargin ||
		yL >= ctx->height - ctx->edgeMargin) {
		*outP = *inP;
		SetMaskAtFast(info, xL, yL, 0);
		return PF_Err_NONE;
	}

	PF_Boolean isLine = IsTargetColorFloatFast(inP, ctx->targetRF, ctx->targetGF, ctx->targetBF, ctx->toleranceSqF);
	SetMaskAtFast(info, xL, yL, isLine ? 255 : 0);

	switch (info->outputMode) {
		case OUTPUT_MODE_FULL:
			if (isLine) {
				FillLinePixelFloat(info, xL, yL, inP, outP, ctx->targetRF, ctx->targetGF, ctx->targetBF, ctx->toleranceSqF, &ctx->colorAdj);
			} else {
				*outP = *inP;
			}
			break;
		case OUTPUT_MODE_LINE_ONLY:
			if (isLine) {
				FillLinePixelFloat(info, xL, yL, inP, outP, ctx->targetRF, ctx->targetGF, ctx->targetBF, ctx->toleranceSqF, &ctx->colorAdj);
				outP->alpha = 1.0f;
			} else {
				outP->alpha = 0;
				outP->red = 0;
				outP->green = 0;
				outP->blue = 0;
			}
			break;
		case OUTPUT_MODE_BG_ONLY:
			if (isLine) {
				outP->alpha = 0;
				outP->red = 0;
				outP->green = 0;
				outP->blue = 0;
			} else {
				*outP = *inP;
			}
			break;
		default:
			*outP = *inP;
			break;
	}
	return PF_Err_NONE;
}

// ============================================================================
// Optimized Blur Pass with Precomputed Weights
// ============================================================================

typedef struct {
	ColorLinesInfo *info;
	PF_EffectWorld *tempWorld;
	A_long blurRadius;
	A_long blurSize;
} BlurContext;

static PF_Err BlurPass8_Optimized(void *refcon, A_long xL, A_long yL, PF_Pixel8 *inP, PF_Pixel8 *outP) {
	BlurContext *ctx = (BlurContext*)refcon;
	ColorLinesInfo *info = ctx->info;

	if (GetMaskAt(info, xL, yL) == 0) {
		*outP = *inP;
		return PF_Err_NONE;
	}

	A_long blurRadius = ctx->blurRadius;
	A_long blurSize = ctx->blurSize;
	PF_FpLong sumR = 0, sumG = 0, sumB = 0, sumA = 0;
	PF_FpLong totalWeight = 0;

	for (A_long dy = -blurRadius; dy <= blurRadius; dy++) {
		A_long ny = yL + dy;
		if (ny < 0 || ny >= info->maskHeight) continue;

		if (GetMaskAt(info, xL, ny) == 0) {
			// Quick check: if center of row is not masked, likely skip
			PF_Boolean hasAnyMask = FALSE;
			for (A_long dx = -blurRadius; dx <= blurRadius; dx++) {
				if (GetMaskAt(info, xL + dx, ny) != 0) {
					hasAnyMask = TRUE;
					break;
				}
			}
			if (!hasAnyMask) continue;
		}

		PF_Pixel8 *rowPtr = GetRow8(ctx->tempWorld, ny);
		A_long weightRowOffset = (dy + blurRadius) * blurSize;

		for (A_long dx = -blurRadius; dx <= blurRadius; dx++) {
			A_long nx = xL + dx;
			if (nx < 0 || nx >= info->maskWidth) continue;
			if (GetMaskAt(info, nx, ny) == 0) continue;

			PF_Pixel8 *neighbor = rowPtr + nx;
			PF_FpLong weight = g_gaussianWeights[weightRowOffset + dx + blurRadius];
			sumR += neighbor->red * weight;
			sumG += neighbor->green * weight;
			sumB += neighbor->blue * weight;
			sumA += neighbor->alpha * weight;
			totalWeight += weight;
		}
	}

	if (totalWeight > 0) {
		PF_FpLong invWeight = 1.0 / totalWeight;
		outP->red = ClampByte(sumR * invWeight);
		outP->green = ClampByte(sumG * invWeight);
		outP->blue = ClampByte(sumB * invWeight);
		outP->alpha = ClampByte(sumA * invWeight);
	} else {
		*outP = *inP;
	}
	return PF_Err_NONE;
}

static PF_Err BlurPass16_Optimized(void *refcon, A_long xL, A_long yL, PF_Pixel16 *inP, PF_Pixel16 *outP) {
	BlurContext *ctx = (BlurContext*)refcon;
	ColorLinesInfo *info = ctx->info;

	if (GetMaskAt(info, xL, yL) == 0) {
		*outP = *inP;
		return PF_Err_NONE;
	}

	A_long blurRadius = ctx->blurRadius;
	A_long blurSize = ctx->blurSize;
	PF_FpLong sumR = 0, sumG = 0, sumB = 0, sumA = 0;
	PF_FpLong totalWeight = 0;

	for (A_long dy = -blurRadius; dy <= blurRadius; dy++) {
		A_long ny = yL + dy;
		if (ny < 0 || ny >= info->maskHeight) continue;

		PF_Pixel16 *rowPtr = GetRow16(ctx->tempWorld, ny);
		A_long weightRowOffset = (dy + blurRadius) * blurSize;

		for (A_long dx = -blurRadius; dx <= blurRadius; dx++) {
			A_long nx = xL + dx;
			if (nx < 0 || nx >= info->maskWidth) continue;
			if (GetMaskAt(info, nx, ny) == 0) continue;

			PF_Pixel16 *neighbor = rowPtr + nx;
			PF_FpLong weight = g_gaussianWeights[weightRowOffset + dx + blurRadius];
			sumR += neighbor->red * weight;
			sumG += neighbor->green * weight;
			sumB += neighbor->blue * weight;
			sumA += neighbor->alpha * weight;
			totalWeight += weight;
		}
	}

	if (totalWeight > 0) {
		PF_FpLong invWeight = 1.0 / totalWeight;
		outP->red = Clamp16(sumR * invWeight);
		outP->green = Clamp16(sumG * invWeight);
		outP->blue = Clamp16(sumB * invWeight);
		outP->alpha = Clamp16(sumA * invWeight);
	} else {
		*outP = *inP;
	}
	return PF_Err_NONE;
}

static PF_Err BlurPassFloat_Optimized(void *refcon, A_long xL, A_long yL, PF_PixelFloat *inP, PF_PixelFloat *outP) {
	BlurContext *ctx = (BlurContext*)refcon;
	ColorLinesInfo *info = ctx->info;

	if (GetMaskAt(info, xL, yL) == 0) {
		*outP = *inP;
		return PF_Err_NONE;
	}

	A_long blurRadius = ctx->blurRadius;
	A_long blurSize = ctx->blurSize;
	PF_FpLong sumR = 0, sumG = 0, sumB = 0, sumA = 0;
	PF_FpLong totalWeight = 0;

	for (A_long dy = -blurRadius; dy <= blurRadius; dy++) {
		A_long ny = yL + dy;
		if (ny < 0 || ny >= info->maskHeight) continue;

		PF_PixelFloat *rowPtr = GetRowFloat(ctx->tempWorld, ny);
		A_long weightRowOffset = (dy + blurRadius) * blurSize;

		for (A_long dx = -blurRadius; dx <= blurRadius; dx++) {
			A_long nx = xL + dx;
			if (nx < 0 || nx >= info->maskWidth) continue;
			if (GetMaskAt(info, nx, ny) == 0) continue;

			PF_PixelFloat *neighbor = rowPtr + nx;
			PF_FpLong weight = g_gaussianWeights[weightRowOffset + dx + blurRadius];
			sumR += neighbor->red * weight;
			sumG += neighbor->green * weight;
			sumB += neighbor->blue * weight;
			sumA += neighbor->alpha * weight;
			totalWeight += weight;
		}
	}

	if (totalWeight > 0) {
		PF_FpLong invWeight = 1.0 / totalWeight;
		outP->red = (PF_FpShort)(sumR * invWeight);
		outP->green = (PF_FpShort)(sumG * invWeight);
		outP->blue = (PF_FpShort)(sumB * invWeight);
		outP->alpha = (PF_FpShort)(sumA * invWeight);
	} else {
		*outP = *inP;
	}
	return PF_Err_NONE;
}

// ============================================================================
// Plugin Entry Points
// ============================================================================

static PF_Err About(PF_InData *in_data, PF_OutData *out_data, PF_ParamDef *params[], PF_LayerDef *output) {
	PF_SPRINTF(out_data->return_msg, "%s, v%d.%d\r%s", NAME, MAJOR_VERSION, MINOR_VERSION, DESCRIPTION);
	return PF_Err_NONE;
}

static PF_Err GlobalSetup(PF_InData *in_dataP, PF_OutData *out_data, PF_ParamDef *params[], PF_LayerDef *output) {
	out_data->my_version = PF_VERSION(MAJOR_VERSION, MINOR_VERSION, BUG_VERSION, STAGE_VERSION, BUILD_VERSION);
	out_data->out_flags = PF_OutFlag_DEEP_COLOR_AWARE;
	out_data->out_flags2 = PF_OutFlag2_FLOAT_COLOR_AWARE | PF_OutFlag2_SUPPORTS_SMART_RENDER | PF_OutFlag2_SUPPORTS_THREADED_RENDERING;
	return PF_Err_NONE;
}

static PF_Err ParamsSetup(PF_InData *in_data, PF_OutData *out_data, PF_ParamDef *params[], PF_LayerDef *output) {
	PF_Err err = PF_Err_NONE;
	PF_ParamDef def;

	AEFX_CLR_STRUCT(def);
	PF_ADD_TOPIC("Color Selection", COLOR_GROUP_START_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_COLOR("Target Color", 0, 0, 0, TARGET_COLOR_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_FLOAT_SLIDERX("Color Tolerance", TOLERANCE_MIN, TOLERANCE_MAX, TOLERANCE_MIN, TOLERANCE_MAX, TOLERANCE_DFLT, PF_Precision_TENTHS, PF_ValueDisplayFlag_PERCENT, 0, COLOR_TOLERANCE_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_END_TOPIC(COLOR_GROUP_END_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_TOPIC("Fill Settings", FILL_GROUP_START_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_POPUP("Fill Mode", FILL_MODE_NUM_MODES - 1, FILL_MODE_WEIGHTED, "Nearest Pixel|Average|Weighted Average", FILL_MODE_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_SLIDER("Search Radius", SEARCH_RADIUS_MIN, SEARCH_RADIUS_MAX, SEARCH_RADIUS_MIN, SEARCH_RADIUS_MAX, SEARCH_RADIUS_DFLT, SEARCH_RADIUS_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_CHECKBOX("Ignore Transparent", "", TRUE, 0, IGNORE_TRANSPARENT_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_FLOAT_SLIDERX("Sample Blur", SAMPLE_BLUR_MIN, SAMPLE_BLUR_MAX, SAMPLE_BLUR_MIN, SAMPLE_BLUR_MAX, SAMPLE_BLUR_DFLT, PF_Precision_TENTHS, PF_ValueDisplayFlag_NONE, 0, SAMPLE_BLUR_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_END_TOPIC(FILL_GROUP_END_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_TOPIC("Color Adjustments", ADJUST_GROUP_START_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_FLOAT_SLIDERX("Brightness", BRIGHTNESS_MIN, BRIGHTNESS_MAX, BRIGHTNESS_MIN, BRIGHTNESS_MAX, BRIGHTNESS_DFLT, PF_Precision_TENTHS, PF_ValueDisplayFlag_NONE, 0, BRIGHTNESS_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_FLOAT_SLIDERX("Contrast", CONTRAST_MIN, CONTRAST_MAX, CONTRAST_MIN, CONTRAST_MAX, CONTRAST_DFLT, PF_Precision_TENTHS, PF_ValueDisplayFlag_NONE, 0, CONTRAST_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_FLOAT_SLIDERX("Saturation", SATURATION_MIN, SATURATION_MAX, SATURATION_MIN, SATURATION_MAX, SATURATION_DFLT, PF_Precision_TENTHS, PF_ValueDisplayFlag_NONE, 0, SATURATION_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_END_TOPIC(ADJUST_GROUP_END_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_TOPIC("Output", OUTPUT_GROUP_START_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_POPUP("Output Mode", OUTPUT_MODE_NUM_MODES - 1, OUTPUT_MODE_FULL, "Full Image|Lines Only|Background Only", OUTPUT_MODE_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_END_TOPIC(OUTPUT_GROUP_END_DISK_ID);

	out_data->num_params = COLORLINES_NUM_PARAMS;
	return err;
}

static PF_Err PreRender(PF_InData *in_dataP, PF_OutData *out_dataP, PF_PreRenderExtra *extraP) {
	PF_Err err = PF_Err_NONE;
	PF_RenderRequest req = extraP->input->output_request;
	PF_CheckoutResult in_result;

	AEFX_SuiteScoper<PF_HandleSuite1> handleSuite = AEFX_SuiteScoper<PF_HandleSuite1>(in_dataP, kPFHandleSuite, kPFHandleSuiteVersion1, out_dataP);
	PF_Handle infoH = handleSuite->host_new_handle(sizeof(ColorLinesInfo));

	if (infoH) {
		ColorLinesInfo *infoP = reinterpret_cast<ColorLinesInfo*>(handleSuite->host_lock_handle(infoH));
		if (infoP) {
			extraP->output->pre_render_data = infoH;
			AEFX_CLR_STRUCT(*infoP);

			PF_ParamDef param;

			AEFX_CLR_STRUCT(param);
			if (!err) err = PF_CHECKOUT_PARAM(in_dataP, COLORLINES_TARGET_COLOR, in_dataP->current_time, in_dataP->time_step, in_dataP->time_scale, &param);
			if (!err) infoP->targetColor = param.u.cd.value;

			AEFX_CLR_STRUCT(param);
			if (!err) err = PF_CHECKOUT_PARAM(in_dataP, COLORLINES_COLOR_TOLERANCE, in_dataP->current_time, in_dataP->time_step, in_dataP->time_scale, &param);
			if (!err) infoP->tolerance = param.u.fs_d.value;

			AEFX_CLR_STRUCT(param);
			if (!err) err = PF_CHECKOUT_PARAM(in_dataP, COLORLINES_FILL_MODE, in_dataP->current_time, in_dataP->time_step, in_dataP->time_scale, &param);
			if (!err) infoP->fillMode = param.u.pd.value;

			AEFX_CLR_STRUCT(param);
			if (!err) err = PF_CHECKOUT_PARAM(in_dataP, COLORLINES_SEARCH_RADIUS, in_dataP->current_time, in_dataP->time_step, in_dataP->time_scale, &param);
			if (!err) infoP->searchRadius = param.u.sd.value;

			AEFX_CLR_STRUCT(param);
			if (!err) err = PF_CHECKOUT_PARAM(in_dataP, COLORLINES_IGNORE_TRANSPARENT, in_dataP->current_time, in_dataP->time_step, in_dataP->time_scale, &param);
			if (!err) infoP->ignoreTransparent = param.u.bd.value;

			AEFX_CLR_STRUCT(param);
			if (!err) err = PF_CHECKOUT_PARAM(in_dataP, COLORLINES_SAMPLE_BLUR, in_dataP->current_time, in_dataP->time_step, in_dataP->time_scale, &param);
			if (!err) infoP->sampleBlur = param.u.fs_d.value;

			AEFX_CLR_STRUCT(param);
			if (!err) err = PF_CHECKOUT_PARAM(in_dataP, COLORLINES_BRIGHTNESS, in_dataP->current_time, in_dataP->time_step, in_dataP->time_scale, &param);
			if (!err) infoP->brightness = param.u.fs_d.value;

			AEFX_CLR_STRUCT(param);
			if (!err) err = PF_CHECKOUT_PARAM(in_dataP, COLORLINES_CONTRAST, in_dataP->current_time, in_dataP->time_step, in_dataP->time_scale, &param);
			if (!err) infoP->contrast = param.u.fs_d.value;

			AEFX_CLR_STRUCT(param);
			if (!err) err = PF_CHECKOUT_PARAM(in_dataP, COLORLINES_SATURATION, in_dataP->current_time, in_dataP->time_step, in_dataP->time_scale, &param);
			if (!err) infoP->saturation = param.u.fs_d.value;

			AEFX_CLR_STRUCT(param);
			if (!err) err = PF_CHECKOUT_PARAM(in_dataP, COLORLINES_OUTPUT_MODE, in_dataP->current_time, in_dataP->time_step, in_dataP->time_scale, &param);
			if (!err) infoP->outputMode = param.u.pd.value;

			if (!err) {
				err = extraP->cb->checkout_layer(in_dataP->effect_ref, COLORLINES_INPUT, COLORLINES_INPUT, &req, in_dataP->current_time, in_dataP->time_step, in_dataP->time_scale, &in_result);
			}
			if (!err) {
				UnionLRect(&in_result.result_rect, &extraP->output->result_rect);
				UnionLRect(&in_result.max_result_rect, &extraP->output->max_result_rect);
			}
			handleSuite->host_unlock_handle(infoH);
		}
	} else {
		err = PF_Err_OUT_OF_MEMORY;
	}
	return err;
}

static PF_Err SmartRender(PF_InData *in_data, PF_OutData *out_data, PF_SmartRenderExtra *extraP) {
	PF_Err err = PF_Err_NONE;
	PF_EffectWorld *input_worldP = NULL, *output_worldP = NULL;
	PF_EffectWorld tempWorld;
	PF_Boolean tempWorldAllocated = FALSE;

	AEFX_SuiteScoper<PF_HandleSuite1> handleSuite = AEFX_SuiteScoper<PF_HandleSuite1>(in_data, kPFHandleSuite, kPFHandleSuiteVersion1, out_data);
	ColorLinesInfo *infoP = reinterpret_cast<ColorLinesInfo*>(handleSuite->host_lock_handle(reinterpret_cast<PF_Handle>(extraP->input->pre_render_data)));

	if (infoP) {
		if (!err) err = extraP->cb->checkout_layer_pixels(in_data->effect_ref, COLORLINES_INPUT, &input_worldP);
		if (!err) err = extraP->cb->checkout_output(in_data->effect_ref, &output_worldP);

		if (!err && input_worldP && output_worldP) {
			infoP->srcWorld = input_worldP;
			infoP->in_data = in_data;

			// Allocate line mask
			infoP->maskWidth = output_worldP->width;
			infoP->maskHeight = output_worldP->height;
			infoP->maskRowBytes = output_worldP->width;
			infoP->lineMask = (A_u_char*)malloc(infoP->maskWidth * infoP->maskHeight);
			if (infoP->lineMask) {
				memset(infoP->lineMask, 0, infoP->maskWidth * infoP->maskHeight);
			}

			// Initialize processing context with precomputed values
			ProcessingContext ctx;
			InitProcessingContext(&ctx, infoP);

			PF_PixelFormat format = PF_PixelFormat_INVALID;
			AEFX_SuiteScoper<PF_WorldSuite2> wsP = AEFX_SuiteScoper<PF_WorldSuite2>(in_data, kPFWorldSuite, kPFWorldSuiteVersion2, out_data);
			if (!err) err = wsP->PF_GetPixelFormat(input_worldP, &format);

			// First pass: Fill line pixels and build mask
			if (!err) {
				switch (format) {
					case PF_PixelFormat_ARGB32: {
						AEFX_SuiteScoper<PF_Iterate8Suite2> iterSuite = AEFX_SuiteScoper<PF_Iterate8Suite2>(in_data, kPFIterate8Suite, kPFIterate8SuiteVersion2, out_data);
						err = iterSuite->iterate(in_data, 0, output_worldP->height, input_worldP, &output_worldP->extent_hint, (void*)&ctx, FillAndMask8_Optimized, output_worldP);
						break;
					}
					case PF_PixelFormat_ARGB64: {
						AEFX_SuiteScoper<PF_iterate16Suite2> iterSuite = AEFX_SuiteScoper<PF_iterate16Suite2>(in_data, kPFIterate16Suite, kPFIterate16SuiteVersion2, out_data);
						err = iterSuite->iterate(in_data, 0, output_worldP->height, input_worldP, &output_worldP->extent_hint, (void*)&ctx, FillAndMask16_Optimized, output_worldP);
						break;
					}
					case PF_PixelFormat_ARGB128: {
						AEFX_SuiteScoper<PF_iterateFloatSuite2> iterSuite = AEFX_SuiteScoper<PF_iterateFloatSuite2>(in_data, kPFIterateFloatSuite, kPFIterateFloatSuiteVersion2, out_data);
						err = iterSuite->iterate(in_data, 0, output_worldP->height, input_worldP, &output_worldP->extent_hint, (void*)&ctx, FillAndMaskFloat_Optimized, output_worldP);
						break;
					}
					default:
						err = PF_Err_BAD_CALLBACK_PARAM;
						break;
				}
			}

			// Second pass: Apply blur if sampleBlur > 0
			A_long blurRadius = (A_long)(infoP->sampleBlur / 10.0);
			if (!err && blurRadius >= 1 && infoP->lineMask) {
				// Precompute gaussian weights
				PrecomputeGaussianWeights(blurRadius);

				AEFX_SuiteScoper<PF_WorldSuite2> worldSuite = AEFX_SuiteScoper<PF_WorldSuite2>(in_data, kPFWorldSuite, kPFWorldSuiteVersion2, out_data);
				err = worldSuite->PF_NewWorld(in_data->effect_ref, output_worldP->width, output_worldP->height, FALSE, format, &tempWorld);

				if (!err) {
					tempWorldAllocated = TRUE;

					// Copy output to temp world
					for (A_long y = 0; y < output_worldP->height; y++) {
						char *srcRow = (char*)output_worldP->data + y * output_worldP->rowbytes;
						char *dstRow = (char*)tempWorld.data + y * tempWorld.rowbytes;
						memcpy(dstRow, srcRow, output_worldP->rowbytes);
					}

					// Setup blur context
					BlurContext blurCtx;
					blurCtx.info = infoP;
					blurCtx.tempWorld = &tempWorld;
					blurCtx.blurRadius = blurRadius;
					blurCtx.blurSize = blurRadius * 2 + 1;

					// Run blur pass
					switch (format) {
						case PF_PixelFormat_ARGB32: {
							AEFX_SuiteScoper<PF_Iterate8Suite2> iterSuite = AEFX_SuiteScoper<PF_Iterate8Suite2>(in_data, kPFIterate8Suite, kPFIterate8SuiteVersion2, out_data);
							err = iterSuite->iterate(in_data, 0, output_worldP->height, &tempWorld, &output_worldP->extent_hint, (void*)&blurCtx, BlurPass8_Optimized, output_worldP);
							break;
						}
						case PF_PixelFormat_ARGB64: {
							AEFX_SuiteScoper<PF_iterate16Suite2> iterSuite = AEFX_SuiteScoper<PF_iterate16Suite2>(in_data, kPFIterate16Suite, kPFIterate16SuiteVersion2, out_data);
							err = iterSuite->iterate(in_data, 0, output_worldP->height, &tempWorld, &output_worldP->extent_hint, (void*)&blurCtx, BlurPass16_Optimized, output_worldP);
							break;
						}
						case PF_PixelFormat_ARGB128: {
							AEFX_SuiteScoper<PF_iterateFloatSuite2> iterSuite = AEFX_SuiteScoper<PF_iterateFloatSuite2>(in_data, kPFIterateFloatSuite, kPFIterateFloatSuiteVersion2, out_data);
							err = iterSuite->iterate(in_data, 0, output_worldP->height, &tempWorld, &output_worldP->extent_hint, (void*)&blurCtx, BlurPassFloat_Optimized, output_worldP);
							break;
						}
						default:
							err = PF_Err_BAD_CALLBACK_PARAM;
							break;
					}
				}

				if (tempWorldAllocated) {
					worldSuite->PF_DisposeWorld(in_data->effect_ref, &tempWorld);
				}
			}

			// Free line mask
			if (infoP->lineMask) {
				free(infoP->lineMask);
				infoP->lineMask = NULL;
			}
		}
		extraP->cb->checkin_layer_pixels(in_data->effect_ref, COLORLINES_INPUT);
	}
	return err;
}

extern "C" DllExport PF_Err PluginDataEntryFunction2(PF_PluginDataPtr inPtr, PF_PluginDataCB2 inPluginDataCallBackPtr, SPBasicSuite* inSPBasicSuitePtr, const char* inHostName, const char* inHostVersion) {
	PF_Err result = PF_Err_NONE;
	result = PF_REGISTER_EFFECT_EXT2(inPtr, inPluginDataCallBackPtr, "cx_ColorLines", "cx_ColorLines", "CX Animation Tools", AE_RESERVED_INFO, "EffectMain", "");
	return result;
}

PF_Err EffectMain(PF_Cmd cmd, PF_InData *in_dataP, PF_OutData *out_data, PF_ParamDef *params[], PF_LayerDef *output, void *extra) {
	PF_Err err = PF_Err_NONE;
	try {
		switch (cmd) {
			case PF_Cmd_ABOUT: err = About(in_dataP, out_data, params, output); break;
			case PF_Cmd_GLOBAL_SETUP: err = GlobalSetup(in_dataP, out_data, params, output); break;
			case PF_Cmd_PARAMS_SETUP: err = ParamsSetup(in_dataP, out_data, params, output); break;
			case PF_Cmd_SMART_PRE_RENDER: err = PreRender(in_dataP, out_data, (PF_PreRenderExtra*)extra); break;
			case PF_Cmd_SMART_RENDER: err = SmartRender(in_dataP, out_data, (PF_SmartRenderExtra*)extra); break;
		}
	} catch (PF_Err &thrown_err) { err = thrown_err; }
	return err;
}
