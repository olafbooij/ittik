#pragma once
#include<calibration_common.hpp>

// values taken from http://www.cvlibs.net/datasets/kitti/downloads/velodynecalib_S2_factory_flatness_intensity.xml
// as pointed to in http://www.cvlibs.net/datasets/kitti/raw_data.php
namespace nonsi {
  auto kitti__probe_calibration()
  {
    return _Calibration{
    _ProbeCalibration{-5.0704789,   -7.287312,    80.249786,  89.673241,  85.349335, 19.737808,  2.5999999},
    _ProbeCalibration{-2.9607918,   -6.9990368,  137.20041,  140.71321,  142.4733,   19.774553, -2.5999999},
    _ProbeCalibration{ 2.7121143,    0.180617,   100,        107,        101,        20.679979,  2.5999999},
    _ProbeCalibration{ 4.9628944,    0.57880998, 132.78882,  141.17041,  137.25119,  20.729954, -2.5999999},
    _ProbeCalibration{-0.79710281,  -6.675746,   108.75352,  117.56853,  112.30875,  19.81571,   2.5999999},
    _ProbeCalibration{ 1.4592539,   -6.3288889,  133.49846,  140.22871,  140.38919,  19.859804, -2.5999999},
    _ProbeCalibration{-1.5792946,   -8.7114143,  122.96078,  127.83755,  127.01267,  19.555548,  2.5999999},
    _ProbeCalibration{ 0.68123782,  -8.3104696,  137.6821,   142.45416,  140.88554,  19.606993, -2.5999999},
    _ProbeCalibration{ 3.4655483,   -5.958395,   100.4778,   108.21006,  105.91731,  19.906839,  2.5999999},
    _ProbeCalibration{ 5.7106166,   -5.6106009,  122.48971,  132.29811,  126.79624,  19.950935, -2.5999999},
    _ProbeCalibration{ 2.708117,    -8.0580254,  115.26807,  121.60332,  116.33878,  19.639328,  2.5999999},
    _ProbeCalibration{ 4.9553113,   -7.6787701,  137.64709,  141.23903,  141.22711,  19.687834, -2.5999999},
    _ProbeCalibration{-5.0824332,   -3.235882,   108.61004,  117.13869,  111.61416,  20.250784,  2.5999999},
    _ProbeCalibration{-2.8410845,   -2.8738379,  128.50006,  135.84097,  133.6692,   20.296349, -2.5999999},
    _ProbeCalibration{-5.8479395,   -5.2507782,  100.70334,  111.11951,  105.58754,  19.996502,  2.5999999},
    _ProbeCalibration{-3.5962763,   -4.9137921,  130.9599,   138.54716,  135.86427,  20.039125, -2.5999999},
    _ProbeCalibration{-0.85072309,  -2.546633,   111.82377,  120.41167,  114.49424,  20.337505,  2.5999999},
    _ProbeCalibration{ 1.4546897,   -2.207566,   138.72282,  144.79926,  145.26318,  20.380131, -2.5999999},
    _ProbeCalibration{-1.5957572,   -4.5881028,  122.32455,  130.80428,  124.08476,  20.080282,  2.5999999},
    _ProbeCalibration{ 0.6645931,   -4.2155242,  140.27374,  146.30959,  147.23538,  20.127317, -2.5999999},
    _ProbeCalibration{ 3.4167359,   -1.833245,   105.84371,  114.49569,  112.79817,  20.427166,  2.5999999},
    _ProbeCalibration{ 5.6595216,   -1.49388,    134.53227,  142.77802,  140.46512,  20.469791, -2.5999999},
    _ProbeCalibration{ 2.6582236,   -3.935853,   118.33218,  126.80502,  120.96743,  20.162594,  2.5999999},
    _ProbeCalibration{ 4.9000001,   -3.5393341,  137.4301,   144.58284,  143.43156,  20.212568, -2.5999999},
    _ProbeCalibration{-5.0803046,    0.871566,   124.1552,   133.87306,  127.56041,  20.766701,  2.5999999},
    _ProbeCalibration{-2.8516858,    1.304757,   128.77693,  136.15088,  134.10022,  20.821083, -2.5999999},
    _ProbeCalibration{-5.8603926,   -1.201239,   131.07784,  139.16788,  135.01427,  20.506536,  2.5999999},
    _ProbeCalibration{-3.6049912,   -0.80315,    122.94209,  130.0735,   127.42249,  20.556513, -2.5999999},
    _ProbeCalibration{-0.87220359,   1.573966,   114.53565,  122.71866,  116.29494,  20.854891,  2.5999999},
    _ProbeCalibration{ 1.4057015,    1.9367,     135.74356,  143.2498,   139.42438,  20.900455, -2.5999999},
    _ProbeCalibration{-1.6291088,   -0.45182899, 126.33221,  135.90332,  130.65894,  20.600607,  2.5999999},
    _ProbeCalibration{ 0.62309581,  -0.088762,   137.5184,   142.89529,  141.83412,  20.646173, -2.5999999},
    _ProbeCalibration{-7.9968266,  -22.856577,   109.50098,  116.78203,  114.83156,  10.795722,  2.5999999},
    _ProbeCalibration{-4.5520601,  -22.437605,    84.899147,  91.157639,  90.969734, 10.849071, -2.5999999},
    _ProbeCalibration{ 4.354651,   -11.598996,   122.29146,  126.08652,  127.10345,  12.139602,  2.5999999},
    _ProbeCalibration{ 7.6049747,  -10.956945,    98.333916, 106.2481,   103.39256,  12.212004, -2.5999999},
    _ProbeCalibration{-1.1401243,  -21.935509,   108.76869,  113.16074,  107.47829,  10.912581,  2.5999999},
    _ProbeCalibration{ 2.4086945,  -21.318125,    92.505463,  96.47541,   96.267891, 10.990064, -2.5999999},
    _ProbeCalibration{-2.3721526,  -24.999201,   122.65831,  121.06113,  119.97446,  10.517547,  2.5999999},
    _ProbeCalibration{ 1.1519399,  -24.506605,    79.283134,  78.707504,  81.518532, 10.582327, -2.5999999},
    _ProbeCalibration{ 5.77879,    -20.787691,   106.13865,  109.20596,  110.5475,   11.056115,  2.5999999},
    _ProbeCalibration{ 9.1956568,  -20.222572,    93.253181,  95.831955,  98.723663, 11.125976, -2.5999999},
    _ProbeCalibration{ 4.5604758,  -23.971016,   113.70682,  115.65723,  114.90288,  10.652189,  2.5999999},
    _ProbeCalibration{ 8.1412535,  -23.322384,    94.428932,  97.686722, 100.81699,  10.736023, -2.5999999},
    _ProbeCalibration{-7.856997,   -16.55401,    119.13224,  124.70088,  125.58075,  11.568009,  2.5999999},
    _ProbeCalibration{-4.4215517,  -16.176632,    99.266823, 108.67934,  107.6288,   11.612466, -2.5999999},
    _ProbeCalibration{-9.1637096,  -19.736372,   130.2906,   129.29901,  131.0228,   11.185676,  2.5999999},
    _ProbeCalibration{-5.6806865,  -19.320236,    93.804993 , 98.647186, 100.48823,  11.236484, -2.5999999},
    _ProbeCalibration{-1.1443137,  -15.656734,   106.6689,   116.74465,  113.83967,  11.673436,  2.5999999},
    _ProbeCalibration{ 2.2717576,  -15.188733,   100.48598,  106.24138,  107.39927,  11.728055, -2.5999999},
    _ProbeCalibration{-2.3783834,  -18.797075,   130.77196,  133.74886,  133.7121,   11.299995,  2.5999999},
    _ProbeCalibration{ 1.1092063,  -18.323431,    94.801849, 101.0146,   101.3172,   11.357154, -2.5999999},
    _ProbeCalibration{ 5.5492816,  -14.598064,   112.19067,  118.50562,  117.49856,  11.796646,  2.5999999},
    _ProbeCalibration{ 8.9375477,  -14.048302,    87.654907 , 94.860596,  92.912048, 11.860156, -2.5999999},
    _ProbeCalibration{ 4.3666,     -17.73037,    130.61176,  132.28065,  130.14856,  11.428286,  2.5999999},
    _ProbeCalibration{ 7.8623509,  -17.186821,    94.141068,  98.307419,  99.442535, 11.493066, -2.5999999},
    _ProbeCalibration{-7.7044425,  -10.470731,   102.647,    112.32704,  107.8148,   12.266623,  2.5999999},
    _ProbeCalibration{-4.3473101,  -10.06249,     96.445595, 101.56944,  102.74843,  12.31235,  -2.5999999},
    _ProbeCalibration{-8.9924641,  -13.484814,   134.63248,  135.41727,  133.99304,  11.924937,  2.5999999},
    _ProbeCalibration{-5.560183,   -13.040989,    94.508636, 100.5687,    99.99778,  11.975745, -2.5999999},
    _ProbeCalibration{-1.0891469,  -9.5735149,   110.23955,  118.49728,  115.68356,  12.366969,  2.5999999},
    _ProbeCalibration{ 2.148736,   -9.0260181,    96.91613,  102.04291,  102.81184,  12.427939, -2.5999999},
    _ProbeCalibration{-2.3586042,  -12.562096,   132.34036,  134.03519,  133.22089,  12.030364,  2.5999999},
    _ProbeCalibration{ 1.0275339,  -12.115005,   107.46816,  110.37566,  104.23552,  12.081173, -2.5999999}
    };
  }
}
auto kitti_probe_calibration()
{
  return common_units(nonsi::kitti__probe_calibration());
}
