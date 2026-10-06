/*
 * multiword_types.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "acf_rl_native".
 *
 * Model version              : 1.48
 * Simulink Coder version : 26.2 (R2026b) 22-May-2026
 * C source code generated on : Mon Oct  5 15:45:43 2026
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef MULTIWORD_TYPES_H
#define MULTIWORD_TYPES_H
#include "rtwtypes.h"

/*
 * MultiWord supporting definitions
 */
typedef long int long_T;

/*
 * MultiWord types
 */
#ifndef INT64M_T
#define INT64M_T

typedef struct {
  uint32_T chunks[2];
} int64m_T;

#endif

#ifndef CINT64M_T
#define CINT64M_T

typedef struct {
  int64m_T re;
  int64m_T im;
} cint64m_T;

#endif

#ifndef UINT64M_T
#define UINT64M_T

typedef struct {
  uint32_T chunks[2];
} uint64m_T;

#endif

#ifndef CUINT64M_T
#define CUINT64M_T

typedef struct {
  uint64m_T re;
  uint64m_T im;
} cuint64m_T;

#endif

#ifndef INT96M_T
#define INT96M_T

typedef struct {
  uint32_T chunks[3];
} int96m_T;

#endif

#ifndef CINT96M_T
#define CINT96M_T

typedef struct {
  int96m_T re;
  int96m_T im;
} cint96m_T;

#endif

#ifndef UINT96M_T
#define UINT96M_T

typedef struct {
  uint32_T chunks[3];
} uint96m_T;

#endif

#ifndef CUINT96M_T
#define CUINT96M_T

typedef struct {
  uint96m_T re;
  uint96m_T im;
} cuint96m_T;

#endif

#ifndef INT128M_T
#define INT128M_T

typedef struct {
  uint32_T chunks[4];
} int128m_T;

#endif

#ifndef CINT128M_T
#define CINT128M_T

typedef struct {
  int128m_T re;
  int128m_T im;
} cint128m_T;

#endif

#ifndef UINT128M_T
#define UINT128M_T

typedef struct {
  uint32_T chunks[4];
} uint128m_T;

#endif

#ifndef CUINT128M_T
#define CUINT128M_T

typedef struct {
  uint128m_T re;
  uint128m_T im;
} cuint128m_T;

#endif

#ifndef INT160M_T
#define INT160M_T

typedef struct {
  uint32_T chunks[5];
} int160m_T;

#endif

#ifndef CINT160M_T
#define CINT160M_T

typedef struct {
  int160m_T re;
  int160m_T im;
} cint160m_T;

#endif

#ifndef UINT160M_T
#define UINT160M_T

typedef struct {
  uint32_T chunks[5];
} uint160m_T;

#endif

#ifndef CUINT160M_T
#define CUINT160M_T

typedef struct {
  uint160m_T re;
  uint160m_T im;
} cuint160m_T;

#endif

#ifndef INT192M_T
#define INT192M_T

typedef struct {
  uint32_T chunks[6];
} int192m_T;

#endif

#ifndef CINT192M_T
#define CINT192M_T

typedef struct {
  int192m_T re;
  int192m_T im;
} cint192m_T;

#endif

#ifndef UINT192M_T
#define UINT192M_T

typedef struct {
  uint32_T chunks[6];
} uint192m_T;

#endif

#ifndef CUINT192M_T
#define CUINT192M_T

typedef struct {
  uint192m_T re;
  uint192m_T im;
} cuint192m_T;

#endif

#ifndef INT224M_T
#define INT224M_T

typedef struct {
  uint32_T chunks[7];
} int224m_T;

#endif

#ifndef CINT224M_T
#define CINT224M_T

typedef struct {
  int224m_T re;
  int224m_T im;
} cint224m_T;

#endif

#ifndef UINT224M_T
#define UINT224M_T

typedef struct {
  uint32_T chunks[7];
} uint224m_T;

#endif

#ifndef CUINT224M_T
#define CUINT224M_T

typedef struct {
  uint224m_T re;
  uint224m_T im;
} cuint224m_T;

#endif

#ifndef INT256M_T
#define INT256M_T

typedef struct {
  uint32_T chunks[8];
} int256m_T;

#endif

#ifndef CINT256M_T
#define CINT256M_T

typedef struct {
  int256m_T re;
  int256m_T im;
} cint256m_T;

#endif

#ifndef UINT256M_T
#define UINT256M_T

typedef struct {
  uint32_T chunks[8];
} uint256m_T;

#endif

#ifndef CUINT256M_T
#define CUINT256M_T

typedef struct {
  uint256m_T re;
  uint256m_T im;
} cuint256m_T;

#endif

#ifndef INT288M_T
#define INT288M_T

typedef struct {
  uint32_T chunks[9];
} int288m_T;

#endif

#ifndef CINT288M_T
#define CINT288M_T

typedef struct {
  int288m_T re;
  int288m_T im;
} cint288m_T;

#endif

#ifndef UINT288M_T
#define UINT288M_T

typedef struct {
  uint32_T chunks[9];
} uint288m_T;

#endif

#ifndef CUINT288M_T
#define CUINT288M_T

typedef struct {
  uint288m_T re;
  uint288m_T im;
} cuint288m_T;

#endif

#ifndef INT320M_T
#define INT320M_T

typedef struct {
  uint32_T chunks[10];
} int320m_T;

#endif

#ifndef CINT320M_T
#define CINT320M_T

typedef struct {
  int320m_T re;
  int320m_T im;
} cint320m_T;

#endif

#ifndef UINT320M_T
#define UINT320M_T

typedef struct {
  uint32_T chunks[10];
} uint320m_T;

#endif

#ifndef CUINT320M_T
#define CUINT320M_T

typedef struct {
  uint320m_T re;
  uint320m_T im;
} cuint320m_T;

#endif

#ifndef INT352M_T
#define INT352M_T

typedef struct {
  uint32_T chunks[11];
} int352m_T;

#endif

#ifndef CINT352M_T
#define CINT352M_T

typedef struct {
  int352m_T re;
  int352m_T im;
} cint352m_T;

#endif

#ifndef UINT352M_T
#define UINT352M_T

typedef struct {
  uint32_T chunks[11];
} uint352m_T;

#endif

#ifndef CUINT352M_T
#define CUINT352M_T

typedef struct {
  uint352m_T re;
  uint352m_T im;
} cuint352m_T;

#endif

#ifndef INT384M_T
#define INT384M_T

typedef struct {
  uint32_T chunks[12];
} int384m_T;

#endif

#ifndef CINT384M_T
#define CINT384M_T

typedef struct {
  int384m_T re;
  int384m_T im;
} cint384m_T;

#endif

#ifndef UINT384M_T
#define UINT384M_T

typedef struct {
  uint32_T chunks[12];
} uint384m_T;

#endif

#ifndef CUINT384M_T
#define CUINT384M_T

typedef struct {
  uint384m_T re;
  uint384m_T im;
} cuint384m_T;

#endif

#ifndef INT416M_T
#define INT416M_T

typedef struct {
  uint32_T chunks[13];
} int416m_T;

#endif

#ifndef CINT416M_T
#define CINT416M_T

typedef struct {
  int416m_T re;
  int416m_T im;
} cint416m_T;

#endif

#ifndef UINT416M_T
#define UINT416M_T

typedef struct {
  uint32_T chunks[13];
} uint416m_T;

#endif

#ifndef CUINT416M_T
#define CUINT416M_T

typedef struct {
  uint416m_T re;
  uint416m_T im;
} cuint416m_T;

#endif

#ifndef INT448M_T
#define INT448M_T

typedef struct {
  uint32_T chunks[14];
} int448m_T;

#endif

#ifndef CINT448M_T
#define CINT448M_T

typedef struct {
  int448m_T re;
  int448m_T im;
} cint448m_T;

#endif

#ifndef UINT448M_T
#define UINT448M_T

typedef struct {
  uint32_T chunks[14];
} uint448m_T;

#endif

#ifndef CUINT448M_T
#define CUINT448M_T

typedef struct {
  uint448m_T re;
  uint448m_T im;
} cuint448m_T;

#endif

#ifndef INT480M_T
#define INT480M_T

typedef struct {
  uint32_T chunks[15];
} int480m_T;

#endif

#ifndef CINT480M_T
#define CINT480M_T

typedef struct {
  int480m_T re;
  int480m_T im;
} cint480m_T;

#endif

#ifndef UINT480M_T
#define UINT480M_T

typedef struct {
  uint32_T chunks[15];
} uint480m_T;

#endif

#ifndef CUINT480M_T
#define CUINT480M_T

typedef struct {
  uint480m_T re;
  uint480m_T im;
} cuint480m_T;

#endif

#ifndef INT512M_T
#define INT512M_T

typedef struct {
  uint32_T chunks[16];
} int512m_T;

#endif

#ifndef CINT512M_T
#define CINT512M_T

typedef struct {
  int512m_T re;
  int512m_T im;
} cint512m_T;

#endif

#ifndef UINT512M_T
#define UINT512M_T

typedef struct {
  uint32_T chunks[16];
} uint512m_T;

#endif

#ifndef CUINT512M_T
#define CUINT512M_T

typedef struct {
  uint512m_T re;
  uint512m_T im;
} cuint512m_T;

#endif

#ifndef INT544M_T
#define INT544M_T

typedef struct {
  uint32_T chunks[17];
} int544m_T;

#endif

#ifndef CINT544M_T
#define CINT544M_T

typedef struct {
  int544m_T re;
  int544m_T im;
} cint544m_T;

#endif

#ifndef UINT544M_T
#define UINT544M_T

typedef struct {
  uint32_T chunks[17];
} uint544m_T;

#endif

#ifndef CUINT544M_T
#define CUINT544M_T

typedef struct {
  uint544m_T re;
  uint544m_T im;
} cuint544m_T;

#endif

#ifndef INT576M_T
#define INT576M_T

typedef struct {
  uint32_T chunks[18];
} int576m_T;

#endif

#ifndef CINT576M_T
#define CINT576M_T

typedef struct {
  int576m_T re;
  int576m_T im;
} cint576m_T;

#endif

#ifndef UINT576M_T
#define UINT576M_T

typedef struct {
  uint32_T chunks[18];
} uint576m_T;

#endif

#ifndef CUINT576M_T
#define CUINT576M_T

typedef struct {
  uint576m_T re;
  uint576m_T im;
} cuint576m_T;

#endif

#ifndef INT608M_T
#define INT608M_T

typedef struct {
  uint32_T chunks[19];
} int608m_T;

#endif

#ifndef CINT608M_T
#define CINT608M_T

typedef struct {
  int608m_T re;
  int608m_T im;
} cint608m_T;

#endif

#ifndef UINT608M_T
#define UINT608M_T

typedef struct {
  uint32_T chunks[19];
} uint608m_T;

#endif

#ifndef CUINT608M_T
#define CUINT608M_T

typedef struct {
  uint608m_T re;
  uint608m_T im;
} cuint608m_T;

#endif

#ifndef INT640M_T
#define INT640M_T

typedef struct {
  uint32_T chunks[20];
} int640m_T;

#endif

#ifndef CINT640M_T
#define CINT640M_T

typedef struct {
  int640m_T re;
  int640m_T im;
} cint640m_T;

#endif

#ifndef UINT640M_T
#define UINT640M_T

typedef struct {
  uint32_T chunks[20];
} uint640m_T;

#endif

#ifndef CUINT640M_T
#define CUINT640M_T

typedef struct {
  uint640m_T re;
  uint640m_T im;
} cuint640m_T;

#endif

#ifndef INT672M_T
#define INT672M_T

typedef struct {
  uint32_T chunks[21];
} int672m_T;

#endif

#ifndef CINT672M_T
#define CINT672M_T

typedef struct {
  int672m_T re;
  int672m_T im;
} cint672m_T;

#endif

#ifndef UINT672M_T
#define UINT672M_T

typedef struct {
  uint32_T chunks[21];
} uint672m_T;

#endif

#ifndef CUINT672M_T
#define CUINT672M_T

typedef struct {
  uint672m_T re;
  uint672m_T im;
} cuint672m_T;

#endif

#ifndef INT704M_T
#define INT704M_T

typedef struct {
  uint32_T chunks[22];
} int704m_T;

#endif

#ifndef CINT704M_T
#define CINT704M_T

typedef struct {
  int704m_T re;
  int704m_T im;
} cint704m_T;

#endif

#ifndef UINT704M_T
#define UINT704M_T

typedef struct {
  uint32_T chunks[22];
} uint704m_T;

#endif

#ifndef CUINT704M_T
#define CUINT704M_T

typedef struct {
  uint704m_T re;
  uint704m_T im;
} cuint704m_T;

#endif

#ifndef INT736M_T
#define INT736M_T

typedef struct {
  uint32_T chunks[23];
} int736m_T;

#endif

#ifndef CINT736M_T
#define CINT736M_T

typedef struct {
  int736m_T re;
  int736m_T im;
} cint736m_T;

#endif

#ifndef UINT736M_T
#define UINT736M_T

typedef struct {
  uint32_T chunks[23];
} uint736m_T;

#endif

#ifndef CUINT736M_T
#define CUINT736M_T

typedef struct {
  uint736m_T re;
  uint736m_T im;
} cuint736m_T;

#endif

#ifndef INT768M_T
#define INT768M_T

typedef struct {
  uint32_T chunks[24];
} int768m_T;

#endif

#ifndef CINT768M_T
#define CINT768M_T

typedef struct {
  int768m_T re;
  int768m_T im;
} cint768m_T;

#endif

#ifndef UINT768M_T
#define UINT768M_T

typedef struct {
  uint32_T chunks[24];
} uint768m_T;

#endif

#ifndef CUINT768M_T
#define CUINT768M_T

typedef struct {
  uint768m_T re;
  uint768m_T im;
} cuint768m_T;

#endif

#ifndef INT800M_T
#define INT800M_T

typedef struct {
  uint32_T chunks[25];
} int800m_T;

#endif

#ifndef CINT800M_T
#define CINT800M_T

typedef struct {
  int800m_T re;
  int800m_T im;
} cint800m_T;

#endif

#ifndef UINT800M_T
#define UINT800M_T

typedef struct {
  uint32_T chunks[25];
} uint800m_T;

#endif

#ifndef CUINT800M_T
#define CUINT800M_T

typedef struct {
  uint800m_T re;
  uint800m_T im;
} cuint800m_T;

#endif

#ifndef INT832M_T
#define INT832M_T

typedef struct {
  uint32_T chunks[26];
} int832m_T;

#endif

#ifndef CINT832M_T
#define CINT832M_T

typedef struct {
  int832m_T re;
  int832m_T im;
} cint832m_T;

#endif

#ifndef UINT832M_T
#define UINT832M_T

typedef struct {
  uint32_T chunks[26];
} uint832m_T;

#endif

#ifndef CUINT832M_T
#define CUINT832M_T

typedef struct {
  uint832m_T re;
  uint832m_T im;
} cuint832m_T;

#endif

#ifndef INT864M_T
#define INT864M_T

typedef struct {
  uint32_T chunks[27];
} int864m_T;

#endif

#ifndef CINT864M_T
#define CINT864M_T

typedef struct {
  int864m_T re;
  int864m_T im;
} cint864m_T;

#endif

#ifndef UINT864M_T
#define UINT864M_T

typedef struct {
  uint32_T chunks[27];
} uint864m_T;

#endif

#ifndef CUINT864M_T
#define CUINT864M_T

typedef struct {
  uint864m_T re;
  uint864m_T im;
} cuint864m_T;

#endif

#ifndef INT896M_T
#define INT896M_T

typedef struct {
  uint32_T chunks[28];
} int896m_T;

#endif

#ifndef CINT896M_T
#define CINT896M_T

typedef struct {
  int896m_T re;
  int896m_T im;
} cint896m_T;

#endif

#ifndef UINT896M_T
#define UINT896M_T

typedef struct {
  uint32_T chunks[28];
} uint896m_T;

#endif

#ifndef CUINT896M_T
#define CUINT896M_T

typedef struct {
  uint896m_T re;
  uint896m_T im;
} cuint896m_T;

#endif

#ifndef INT928M_T
#define INT928M_T

typedef struct {
  uint32_T chunks[29];
} int928m_T;

#endif

#ifndef CINT928M_T
#define CINT928M_T

typedef struct {
  int928m_T re;
  int928m_T im;
} cint928m_T;

#endif

#ifndef UINT928M_T
#define UINT928M_T

typedef struct {
  uint32_T chunks[29];
} uint928m_T;

#endif

#ifndef CUINT928M_T
#define CUINT928M_T

typedef struct {
  uint928m_T re;
  uint928m_T im;
} cuint928m_T;

#endif

#ifndef INT960M_T
#define INT960M_T

typedef struct {
  uint32_T chunks[30];
} int960m_T;

#endif

#ifndef CINT960M_T
#define CINT960M_T

typedef struct {
  int960m_T re;
  int960m_T im;
} cint960m_T;

#endif

#ifndef UINT960M_T
#define UINT960M_T

typedef struct {
  uint32_T chunks[30];
} uint960m_T;

#endif

#ifndef CUINT960M_T
#define CUINT960M_T

typedef struct {
  uint960m_T re;
  uint960m_T im;
} cuint960m_T;

#endif

#ifndef INT992M_T
#define INT992M_T

typedef struct {
  uint32_T chunks[31];
} int992m_T;

#endif

#ifndef CINT992M_T
#define CINT992M_T

typedef struct {
  int992m_T re;
  int992m_T im;
} cint992m_T;

#endif

#ifndef UINT992M_T
#define UINT992M_T

typedef struct {
  uint32_T chunks[31];
} uint992m_T;

#endif

#ifndef CUINT992M_T
#define CUINT992M_T

typedef struct {
  uint992m_T re;
  uint992m_T im;
} cuint992m_T;

#endif

#ifndef INT1024M_T
#define INT1024M_T

typedef struct {
  uint32_T chunks[32];
} int1024m_T;

#endif

#ifndef CINT1024M_T
#define CINT1024M_T

typedef struct {
  int1024m_T re;
  int1024m_T im;
} cint1024m_T;

#endif

#ifndef UINT1024M_T
#define UINT1024M_T

typedef struct {
  uint32_T chunks[32];
} uint1024m_T;

#endif

#ifndef CUINT1024M_T
#define CUINT1024M_T

typedef struct {
  uint1024m_T re;
  uint1024m_T im;
} cuint1024m_T;

#endif

#ifndef INT1056M_T
#define INT1056M_T

typedef struct {
  uint32_T chunks[33];
} int1056m_T;

#endif

#ifndef CINT1056M_T
#define CINT1056M_T

typedef struct {
  int1056m_T re;
  int1056m_T im;
} cint1056m_T;

#endif

#ifndef UINT1056M_T
#define UINT1056M_T

typedef struct {
  uint32_T chunks[33];
} uint1056m_T;

#endif

#ifndef CUINT1056M_T
#define CUINT1056M_T

typedef struct {
  uint1056m_T re;
  uint1056m_T im;
} cuint1056m_T;

#endif

#ifndef INT1088M_T
#define INT1088M_T

typedef struct {
  uint32_T chunks[34];
} int1088m_T;

#endif

#ifndef CINT1088M_T
#define CINT1088M_T

typedef struct {
  int1088m_T re;
  int1088m_T im;
} cint1088m_T;

#endif

#ifndef UINT1088M_T
#define UINT1088M_T

typedef struct {
  uint32_T chunks[34];
} uint1088m_T;

#endif

#ifndef CUINT1088M_T
#define CUINT1088M_T

typedef struct {
  uint1088m_T re;
  uint1088m_T im;
} cuint1088m_T;

#endif

#ifndef INT1120M_T
#define INT1120M_T

typedef struct {
  uint32_T chunks[35];
} int1120m_T;

#endif

#ifndef CINT1120M_T
#define CINT1120M_T

typedef struct {
  int1120m_T re;
  int1120m_T im;
} cint1120m_T;

#endif

#ifndef UINT1120M_T
#define UINT1120M_T

typedef struct {
  uint32_T chunks[35];
} uint1120m_T;

#endif

#ifndef CUINT1120M_T
#define CUINT1120M_T

typedef struct {
  uint1120m_T re;
  uint1120m_T im;
} cuint1120m_T;

#endif

#ifndef INT1152M_T
#define INT1152M_T

typedef struct {
  uint32_T chunks[36];
} int1152m_T;

#endif

#ifndef CINT1152M_T
#define CINT1152M_T

typedef struct {
  int1152m_T re;
  int1152m_T im;
} cint1152m_T;

#endif

#ifndef UINT1152M_T
#define UINT1152M_T

typedef struct {
  uint32_T chunks[36];
} uint1152m_T;

#endif

#ifndef CUINT1152M_T
#define CUINT1152M_T

typedef struct {
  uint1152m_T re;
  uint1152m_T im;
} cuint1152m_T;

#endif

#ifndef INT1184M_T
#define INT1184M_T

typedef struct {
  uint32_T chunks[37];
} int1184m_T;

#endif

#ifndef CINT1184M_T
#define CINT1184M_T

typedef struct {
  int1184m_T re;
  int1184m_T im;
} cint1184m_T;

#endif

#ifndef UINT1184M_T
#define UINT1184M_T

typedef struct {
  uint32_T chunks[37];
} uint1184m_T;

#endif

#ifndef CUINT1184M_T
#define CUINT1184M_T

typedef struct {
  uint1184m_T re;
  uint1184m_T im;
} cuint1184m_T;

#endif

#ifndef INT1216M_T
#define INT1216M_T

typedef struct {
  uint32_T chunks[38];
} int1216m_T;

#endif

#ifndef CINT1216M_T
#define CINT1216M_T

typedef struct {
  int1216m_T re;
  int1216m_T im;
} cint1216m_T;

#endif

#ifndef UINT1216M_T
#define UINT1216M_T

typedef struct {
  uint32_T chunks[38];
} uint1216m_T;

#endif

#ifndef CUINT1216M_T
#define CUINT1216M_T

typedef struct {
  uint1216m_T re;
  uint1216m_T im;
} cuint1216m_T;

#endif

#ifndef INT1248M_T
#define INT1248M_T

typedef struct {
  uint32_T chunks[39];
} int1248m_T;

#endif

#ifndef CINT1248M_T
#define CINT1248M_T

typedef struct {
  int1248m_T re;
  int1248m_T im;
} cint1248m_T;

#endif

#ifndef UINT1248M_T
#define UINT1248M_T

typedef struct {
  uint32_T chunks[39];
} uint1248m_T;

#endif

#ifndef CUINT1248M_T
#define CUINT1248M_T

typedef struct {
  uint1248m_T re;
  uint1248m_T im;
} cuint1248m_T;

#endif

#ifndef INT1280M_T
#define INT1280M_T

typedef struct {
  uint32_T chunks[40];
} int1280m_T;

#endif

#ifndef CINT1280M_T
#define CINT1280M_T

typedef struct {
  int1280m_T re;
  int1280m_T im;
} cint1280m_T;

#endif

#ifndef UINT1280M_T
#define UINT1280M_T

typedef struct {
  uint32_T chunks[40];
} uint1280m_T;

#endif

#ifndef CUINT1280M_T
#define CUINT1280M_T

typedef struct {
  uint1280m_T re;
  uint1280m_T im;
} cuint1280m_T;

#endif

#ifndef INT1312M_T
#define INT1312M_T

typedef struct {
  uint32_T chunks[41];
} int1312m_T;

#endif

#ifndef CINT1312M_T
#define CINT1312M_T

typedef struct {
  int1312m_T re;
  int1312m_T im;
} cint1312m_T;

#endif

#ifndef UINT1312M_T
#define UINT1312M_T

typedef struct {
  uint32_T chunks[41];
} uint1312m_T;

#endif

#ifndef CUINT1312M_T
#define CUINT1312M_T

typedef struct {
  uint1312m_T re;
  uint1312m_T im;
} cuint1312m_T;

#endif

#ifndef INT1344M_T
#define INT1344M_T

typedef struct {
  uint32_T chunks[42];
} int1344m_T;

#endif

#ifndef CINT1344M_T
#define CINT1344M_T

typedef struct {
  int1344m_T re;
  int1344m_T im;
} cint1344m_T;

#endif

#ifndef UINT1344M_T
#define UINT1344M_T

typedef struct {
  uint32_T chunks[42];
} uint1344m_T;

#endif

#ifndef CUINT1344M_T
#define CUINT1344M_T

typedef struct {
  uint1344m_T re;
  uint1344m_T im;
} cuint1344m_T;

#endif

#ifndef INT1376M_T
#define INT1376M_T

typedef struct {
  uint32_T chunks[43];
} int1376m_T;

#endif

#ifndef CINT1376M_T
#define CINT1376M_T

typedef struct {
  int1376m_T re;
  int1376m_T im;
} cint1376m_T;

#endif

#ifndef UINT1376M_T
#define UINT1376M_T

typedef struct {
  uint32_T chunks[43];
} uint1376m_T;

#endif

#ifndef CUINT1376M_T
#define CUINT1376M_T

typedef struct {
  uint1376m_T re;
  uint1376m_T im;
} cuint1376m_T;

#endif

#ifndef INT1408M_T
#define INT1408M_T

typedef struct {
  uint32_T chunks[44];
} int1408m_T;

#endif

#ifndef CINT1408M_T
#define CINT1408M_T

typedef struct {
  int1408m_T re;
  int1408m_T im;
} cint1408m_T;

#endif

#ifndef UINT1408M_T
#define UINT1408M_T

typedef struct {
  uint32_T chunks[44];
} uint1408m_T;

#endif

#ifndef CUINT1408M_T
#define CUINT1408M_T

typedef struct {
  uint1408m_T re;
  uint1408m_T im;
} cuint1408m_T;

#endif

#ifndef INT1440M_T
#define INT1440M_T

typedef struct {
  uint32_T chunks[45];
} int1440m_T;

#endif

#ifndef CINT1440M_T
#define CINT1440M_T

typedef struct {
  int1440m_T re;
  int1440m_T im;
} cint1440m_T;

#endif

#ifndef UINT1440M_T
#define UINT1440M_T

typedef struct {
  uint32_T chunks[45];
} uint1440m_T;

#endif

#ifndef CUINT1440M_T
#define CUINT1440M_T

typedef struct {
  uint1440m_T re;
  uint1440m_T im;
} cuint1440m_T;

#endif

#ifndef INT1472M_T
#define INT1472M_T

typedef struct {
  uint32_T chunks[46];
} int1472m_T;

#endif

#ifndef CINT1472M_T
#define CINT1472M_T

typedef struct {
  int1472m_T re;
  int1472m_T im;
} cint1472m_T;

#endif

#ifndef UINT1472M_T
#define UINT1472M_T

typedef struct {
  uint32_T chunks[46];
} uint1472m_T;

#endif

#ifndef CUINT1472M_T
#define CUINT1472M_T

typedef struct {
  uint1472m_T re;
  uint1472m_T im;
} cuint1472m_T;

#endif

#ifndef INT1504M_T
#define INT1504M_T

typedef struct {
  uint32_T chunks[47];
} int1504m_T;

#endif

#ifndef CINT1504M_T
#define CINT1504M_T

typedef struct {
  int1504m_T re;
  int1504m_T im;
} cint1504m_T;

#endif

#ifndef UINT1504M_T
#define UINT1504M_T

typedef struct {
  uint32_T chunks[47];
} uint1504m_T;

#endif

#ifndef CUINT1504M_T
#define CUINT1504M_T

typedef struct {
  uint1504m_T re;
  uint1504m_T im;
} cuint1504m_T;

#endif

#ifndef INT1536M_T
#define INT1536M_T

typedef struct {
  uint32_T chunks[48];
} int1536m_T;

#endif

#ifndef CINT1536M_T
#define CINT1536M_T

typedef struct {
  int1536m_T re;
  int1536m_T im;
} cint1536m_T;

#endif

#ifndef UINT1536M_T
#define UINT1536M_T

typedef struct {
  uint32_T chunks[48];
} uint1536m_T;

#endif

#ifndef CUINT1536M_T
#define CUINT1536M_T

typedef struct {
  uint1536m_T re;
  uint1536m_T im;
} cuint1536m_T;

#endif

#ifndef INT1568M_T
#define INT1568M_T

typedef struct {
  uint32_T chunks[49];
} int1568m_T;

#endif

#ifndef CINT1568M_T
#define CINT1568M_T

typedef struct {
  int1568m_T re;
  int1568m_T im;
} cint1568m_T;

#endif

#ifndef UINT1568M_T
#define UINT1568M_T

typedef struct {
  uint32_T chunks[49];
} uint1568m_T;

#endif

#ifndef CUINT1568M_T
#define CUINT1568M_T

typedef struct {
  uint1568m_T re;
  uint1568m_T im;
} cuint1568m_T;

#endif

#ifndef INT1600M_T
#define INT1600M_T

typedef struct {
  uint32_T chunks[50];
} int1600m_T;

#endif

#ifndef CINT1600M_T
#define CINT1600M_T

typedef struct {
  int1600m_T re;
  int1600m_T im;
} cint1600m_T;

#endif

#ifndef UINT1600M_T
#define UINT1600M_T

typedef struct {
  uint32_T chunks[50];
} uint1600m_T;

#endif

#ifndef CUINT1600M_T
#define CUINT1600M_T

typedef struct {
  uint1600m_T re;
  uint1600m_T im;
} cuint1600m_T;

#endif

#ifndef INT1632M_T
#define INT1632M_T

typedef struct {
  uint32_T chunks[51];
} int1632m_T;

#endif

#ifndef CINT1632M_T
#define CINT1632M_T

typedef struct {
  int1632m_T re;
  int1632m_T im;
} cint1632m_T;

#endif

#ifndef UINT1632M_T
#define UINT1632M_T

typedef struct {
  uint32_T chunks[51];
} uint1632m_T;

#endif

#ifndef CUINT1632M_T
#define CUINT1632M_T

typedef struct {
  uint1632m_T re;
  uint1632m_T im;
} cuint1632m_T;

#endif

#ifndef INT1664M_T
#define INT1664M_T

typedef struct {
  uint32_T chunks[52];
} int1664m_T;

#endif

#ifndef CINT1664M_T
#define CINT1664M_T

typedef struct {
  int1664m_T re;
  int1664m_T im;
} cint1664m_T;

#endif

#ifndef UINT1664M_T
#define UINT1664M_T

typedef struct {
  uint32_T chunks[52];
} uint1664m_T;

#endif

#ifndef CUINT1664M_T
#define CUINT1664M_T

typedef struct {
  uint1664m_T re;
  uint1664m_T im;
} cuint1664m_T;

#endif

#ifndef INT1696M_T
#define INT1696M_T

typedef struct {
  uint32_T chunks[53];
} int1696m_T;

#endif

#ifndef CINT1696M_T
#define CINT1696M_T

typedef struct {
  int1696m_T re;
  int1696m_T im;
} cint1696m_T;

#endif

#ifndef UINT1696M_T
#define UINT1696M_T

typedef struct {
  uint32_T chunks[53];
} uint1696m_T;

#endif

#ifndef CUINT1696M_T
#define CUINT1696M_T

typedef struct {
  uint1696m_T re;
  uint1696m_T im;
} cuint1696m_T;

#endif

#ifndef INT1728M_T
#define INT1728M_T

typedef struct {
  uint32_T chunks[54];
} int1728m_T;

#endif

#ifndef CINT1728M_T
#define CINT1728M_T

typedef struct {
  int1728m_T re;
  int1728m_T im;
} cint1728m_T;

#endif

#ifndef UINT1728M_T
#define UINT1728M_T

typedef struct {
  uint32_T chunks[54];
} uint1728m_T;

#endif

#ifndef CUINT1728M_T
#define CUINT1728M_T

typedef struct {
  uint1728m_T re;
  uint1728m_T im;
} cuint1728m_T;

#endif

#ifndef INT1760M_T
#define INT1760M_T

typedef struct {
  uint32_T chunks[55];
} int1760m_T;

#endif

#ifndef CINT1760M_T
#define CINT1760M_T

typedef struct {
  int1760m_T re;
  int1760m_T im;
} cint1760m_T;

#endif

#ifndef UINT1760M_T
#define UINT1760M_T

typedef struct {
  uint32_T chunks[55];
} uint1760m_T;

#endif

#ifndef CUINT1760M_T
#define CUINT1760M_T

typedef struct {
  uint1760m_T re;
  uint1760m_T im;
} cuint1760m_T;

#endif

#ifndef INT1792M_T
#define INT1792M_T

typedef struct {
  uint32_T chunks[56];
} int1792m_T;

#endif

#ifndef CINT1792M_T
#define CINT1792M_T

typedef struct {
  int1792m_T re;
  int1792m_T im;
} cint1792m_T;

#endif

#ifndef UINT1792M_T
#define UINT1792M_T

typedef struct {
  uint32_T chunks[56];
} uint1792m_T;

#endif

#ifndef CUINT1792M_T
#define CUINT1792M_T

typedef struct {
  uint1792m_T re;
  uint1792m_T im;
} cuint1792m_T;

#endif

#ifndef INT1824M_T
#define INT1824M_T

typedef struct {
  uint32_T chunks[57];
} int1824m_T;

#endif

#ifndef CINT1824M_T
#define CINT1824M_T

typedef struct {
  int1824m_T re;
  int1824m_T im;
} cint1824m_T;

#endif

#ifndef UINT1824M_T
#define UINT1824M_T

typedef struct {
  uint32_T chunks[57];
} uint1824m_T;

#endif

#ifndef CUINT1824M_T
#define CUINT1824M_T

typedef struct {
  uint1824m_T re;
  uint1824m_T im;
} cuint1824m_T;

#endif

#ifndef INT1856M_T
#define INT1856M_T

typedef struct {
  uint32_T chunks[58];
} int1856m_T;

#endif

#ifndef CINT1856M_T
#define CINT1856M_T

typedef struct {
  int1856m_T re;
  int1856m_T im;
} cint1856m_T;

#endif

#ifndef UINT1856M_T
#define UINT1856M_T

typedef struct {
  uint32_T chunks[58];
} uint1856m_T;

#endif

#ifndef CUINT1856M_T
#define CUINT1856M_T

typedef struct {
  uint1856m_T re;
  uint1856m_T im;
} cuint1856m_T;

#endif

#ifndef INT1888M_T
#define INT1888M_T

typedef struct {
  uint32_T chunks[59];
} int1888m_T;

#endif

#ifndef CINT1888M_T
#define CINT1888M_T

typedef struct {
  int1888m_T re;
  int1888m_T im;
} cint1888m_T;

#endif

#ifndef UINT1888M_T
#define UINT1888M_T

typedef struct {
  uint32_T chunks[59];
} uint1888m_T;

#endif

#ifndef CUINT1888M_T
#define CUINT1888M_T

typedef struct {
  uint1888m_T re;
  uint1888m_T im;
} cuint1888m_T;

#endif

#ifndef INT1920M_T
#define INT1920M_T

typedef struct {
  uint32_T chunks[60];
} int1920m_T;

#endif

#ifndef CINT1920M_T
#define CINT1920M_T

typedef struct {
  int1920m_T re;
  int1920m_T im;
} cint1920m_T;

#endif

#ifndef UINT1920M_T
#define UINT1920M_T

typedef struct {
  uint32_T chunks[60];
} uint1920m_T;

#endif

#ifndef CUINT1920M_T
#define CUINT1920M_T

typedef struct {
  uint1920m_T re;
  uint1920m_T im;
} cuint1920m_T;

#endif

#ifndef INT1952M_T
#define INT1952M_T

typedef struct {
  uint32_T chunks[61];
} int1952m_T;

#endif

#ifndef CINT1952M_T
#define CINT1952M_T

typedef struct {
  int1952m_T re;
  int1952m_T im;
} cint1952m_T;

#endif

#ifndef UINT1952M_T
#define UINT1952M_T

typedef struct {
  uint32_T chunks[61];
} uint1952m_T;

#endif

#ifndef CUINT1952M_T
#define CUINT1952M_T

typedef struct {
  uint1952m_T re;
  uint1952m_T im;
} cuint1952m_T;

#endif

#ifndef INT1984M_T
#define INT1984M_T

typedef struct {
  uint32_T chunks[62];
} int1984m_T;

#endif

#ifndef CINT1984M_T
#define CINT1984M_T

typedef struct {
  int1984m_T re;
  int1984m_T im;
} cint1984m_T;

#endif

#ifndef UINT1984M_T
#define UINT1984M_T

typedef struct {
  uint32_T chunks[62];
} uint1984m_T;

#endif

#ifndef CUINT1984M_T
#define CUINT1984M_T

typedef struct {
  uint1984m_T re;
  uint1984m_T im;
} cuint1984m_T;

#endif

#ifndef INT2016M_T
#define INT2016M_T

typedef struct {
  uint32_T chunks[63];
} int2016m_T;

#endif

#ifndef CINT2016M_T
#define CINT2016M_T

typedef struct {
  int2016m_T re;
  int2016m_T im;
} cint2016m_T;

#endif

#ifndef UINT2016M_T
#define UINT2016M_T

typedef struct {
  uint32_T chunks[63];
} uint2016m_T;

#endif

#ifndef CUINT2016M_T
#define CUINT2016M_T

typedef struct {
  uint2016m_T re;
  uint2016m_T im;
} cuint2016m_T;

#endif

#ifndef INT2048M_T
#define INT2048M_T

typedef struct {
  uint32_T chunks[64];
} int2048m_T;

#endif

#ifndef CINT2048M_T
#define CINT2048M_T

typedef struct {
  int2048m_T re;
  int2048m_T im;
} cint2048m_T;

#endif

#ifndef UINT2048M_T
#define UINT2048M_T

typedef struct {
  uint32_T chunks[64];
} uint2048m_T;

#endif

#ifndef CUINT2048M_T
#define CUINT2048M_T

typedef struct {
  uint2048m_T re;
  uint2048m_T im;
} cuint2048m_T;

#endif
#endif                                 /* MULTIWORD_TYPES_H */
