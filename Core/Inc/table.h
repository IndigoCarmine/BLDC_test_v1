#pragma once

#include "stdint.h"

#define TABLE_SIZE 427
#define WAVE_SIZE 256
#define U_POINT 0
#define V_POINT 85
#define W_POINT 170

const uint16_t sine_table[427] = {
    32767, 33973, 35176, 36374, 37564, 38744, 39910, 41061, 42194, 43306, 44396, 45462, 46501, 47511, 48490, 49438, 50352, 51231, 52074, 52880, 53648, 54378, 55068, 55718, 56328, 56898, 57429, 57919, 58371,
    58784, 59159, 59497, 59799, 60066, 60299, 60501, 60671, 60812, 60926, 61014, 61078, 61119, 61141, 61144, 61130, 61102, 61061, 61010, 60950, 60883, 60811, 60736, 60659, 60582, 60506, 60433, 60364, 60300, 60243,
    60193, 60151, 60117, 60093, 60078, 60073, 60078, 60093, 60117, 60151, 60193, 60243, 60300, 60364, 60433, 60506, 60582, 60659, 60736, 60811, 60883, 60950, 61010, 61061, 61102, 61130, 61144, 61141, 61119, 61078,
    61014, 60926, 60812, 60671, 60501, 60299, 60066, 59799, 59497, 59159, 58784, 58371, 57919, 57429, 56898, 56328, 55718, 55068, 54378, 53648, 52880, 52074, 51231, 50352, 49438, 48490, 47511, 46501, 45462, 44396,
    43306, 42194, 41061, 39910, 38744, 37564, 36374, 35176, 33973, 32767, 31561, 30358, 29160, 27970, 26790, 25624, 24473, 23340, 22228, 21138, 20072, 19033, 18023, 17044, 16096, 15182, 14303, 13460, 12654, 11886,
    11156, 10466, 9816, 9206, 8636, 8105, 7615, 7163, 6750, 6375, 6037, 5735, 5468, 5235, 5033, 4863, 4722, 4608, 4520, 4456, 4415, 4393, 4390, 4404, 4432, 4473, 4524, 4584, 4651, 4723,
    4798, 4875, 4952, 5028, 5101, 5170, 5234, 5291, 5341, 5383, 5417, 5441, 5456, 5461, 5456, 5441, 5417, 5383, 5341, 5291, 5234, 5170, 5101, 5028, 4952, 4875, 4798, 4723, 4651, 4584,
    4524, 4473, 4432, 4404, 4390, 4393, 4415, 4456, 4520, 4608, 4722, 4863, 5033, 5235, 5468, 5735, 6037, 6375, 6750, 7163, 7615, 8105, 8636, 9206, 9816, 10466, 11156, 11886, 12654, 13460,
    14303, 15182, 16096, 17044, 18023, 19033, 20072, 21138, 22228, 23340, 24473, 25624, 26790, 27970, 29160, 30358, 31561, 32767, 33973, 35176, 36374, 37564, 38744, 39910, 41061, 42194, 43306, 44396, 45462, 46501,
    47511, 48490, 49438, 50352, 51231, 52074, 52880, 53648, 54378, 55068, 55718, 56328, 56898, 57429, 57919, 58371, 58784, 59159, 59497, 59799, 60066, 60299, 60501, 60671, 60812, 60926, 61014, 61078, 61119, 61141,
    61144, 61130, 61102, 61061, 61010, 60950, 60883, 60811, 60736, 60659, 60582, 60506, 60433, 60364, 60300, 60243, 60193, 60151, 60117, 60093, 60078, 60073, 60078, 60093, 60117, 60151, 60193, 60243, 60300, 60364,
    60433, 60506, 60582, 60659, 60736, 60811, 60883, 60950, 61010, 61061, 61102, 61130, 61144, 61141, 61119, 61078, 61014, 60926, 60812, 60671, 60501, 60299, 60066, 59799, 59497, 59159, 58784, 58371, 57919, 57429,
    56898, 56328, 55718, 55068, 54378, 53648, 52880, 52074, 51231, 50352, 49438, 48490, 47511, 46501, 45462, 44396, 43306, 42194, 41061, 39910, 38744, 37564, 36374, 35176, 33973, 32767, 31561, 30358, 29160, 27970,
    26790, 25624, 24473, 23340, 22228, 21138, 20072, 19033, 18023, 17044, 16096, 15182, 14303, 13460, 12654, 11886, 11156, 10466, 9816, 9206, 8636, 8105, 7615, 7163, 6750, 6375, 6037, 5735, 5468, 5235,
    5033, 4863, 4722, 4608, 4520, 4456, 4415, 4393};
