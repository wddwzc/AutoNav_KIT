#ifndef __data_process_hpp__
#define __data_process_hpp__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <cmath>
using namespace Eigen;

#ifndef __pi__
#define __pi__
const double PI = 3.1415926535898;
#endif

typedef geometry_msgs::Vector3 ROSVec3d;
typedef geometry_msgs::Point ROSPoi3d;

/***************经纬高转enu坐标******************/
void BLH2ENU(double lat, double lon, double alt, double &x, double &y,
             double &z, double sin_lat, double cos_lat, double sin_lon,
             double cos_lon) {
  //经度(单位为°),纬度(单位为°),高度(单位m),东北天坐标系下坐标,ECEF->ENU(trans)
  lat = lat * PI / 180; // To rad.
  lon = lon * PI / 180;
  double f = 1 / 298.257223563; // WGS84
  double a = 6378137.0;         // WGS84
  double b = a * (1 - f);
  double e = sqrt(a * a - b * b) / a;
  double N = a / sqrt(1 - e * e * sin(lat) * sin(lat));
  // To ECEF
  double Xe = (N + alt) * cos(lat) * cos(lon); //地心系下坐标(单位m)
  double Ye = (N + alt) * cos(lat) * sin(lon);
  double Ze = (N * (1 - (e * e)) + alt) * sin(lat);
  // To ENU
  x = -Xe * sin_lon + Ye * cos_lon; //东北天坐标系下坐标
  y = -Xe * sin_lat * cos_lon - Ye * sin_lat * sin_lon + Ze * cos_lat;
  z = Xe * cos_lat * cos_lon + Ye * cos_lat * sin_lon + Ze * sin_lat;
}
/******均值滤波算法******/
void AverageFilter(ROSPoi3d &inPos, ROSPoi3d &outPos, int num) {
  static int countIn(0);
  static const int pointNum = num; // 50点
  static float *smX = new float[pointNum];
  static float *smY = new float[pointNum];
  static float *smZ = new float[pointNum];
  if (countIn < pointNum) {
    countIn++;
    smX[countIn - 1] = inPos.x;
    smY[countIn - 1] = inPos.y;
    smZ[countIn - 1] = inPos.z;
    outPos = inPos;
    return;
  }
  for (int i = 0; i < pointNum - 1; ++i) {
    smX[i] = smX[i + 1];
    smY[i] = smY[i + 1];
    smZ[i] = smZ[i + 1];
  }
  smX[pointNum - 1] = inPos.x;
  smY[pointNum - 1] = inPos.y;
  smZ[pointNum - 1] = inPos.z;
  float sumX(0), sumY(0), sumZ(0);
  for (int i = 0; i < pointNum; ++i) {
    sumX += smX[i];
    sumY += smY[i];
    sumZ += smZ[i];
  }
  outPos.x = sumX / pointNum;
  outPos.y = sumY / pointNum;
  outPos.z = sumZ / pointNum;
}
/**************卡尔曼滤波******************/
void KalmanFun(const ROSVec3d &acc, ROSPoi3d &inPos, double dt,
               ROSPoi3d &outPos, int status) {
  //初始化
  static bool isFirstKal(true);
  static MatrixXd A(4, 4);
  static MatrixXd B(4, 2);
  static MatrixXd u(2, 1);
  static MatrixXd Q(4, 4);
  static MatrixXd H(2, 4);
  static MatrixXd R(2, 2);
  static MatrixXd X_pdct(4, 1);
  static MatrixXd Pk_pdct(4, 4);
  static MatrixXd K(4, 2);
  static MatrixXd Z_meas(2, 1);
  static MatrixXd X_evlt(4, 1);
  static MatrixXd Pk_evlt(4, 4);
  if (isFirstKal) {
    //变量定义，包括状态预测值，状态估计值，测量值，预测状态与真实状态的协方差矩阵，
    //估计状态和真实状态的协方差矩阵，初始值均为零
    X_pdct.setZero();
    Pk_pdct.setZero();
    K.setZero();
    Z_meas.setZero(); //测量值
    X_evlt.setZero(); // 状态变量初始值
    Pk_evlt.setZero();
    isFirstKal = false;
  }
  A << 1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1;

  B << 0.5 * dt * dt, 0, 0, 0.5 * dt * dt, dt, 0, 0, dt;

  u << acc.x, acc.y;

  Q << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0,
      0.01; //过程激励噪声协方差

  H << 1, 0, 0, 0, 0, 1, 0,
      0; //测量的是小车的位移，速度为0,速度用delta t内的平均速度
  if (status < 2) {
    R << 1, 0, 0, 1; //观测噪声协方差
  } else if (status == 2) {
    R << 0.5, 0, 0, 0.5; //观测噪声协方差
  } else {
    R << 0.01, 0, 0, 0.01; //观测噪声协方差
  }

  //预测值
  X_pdct = A * X_evlt + B * u;
  //预测状态与真实状态的协方差矩阵，Pk'
  Pk_pdct = A * Pk_evlt * A.transpose() + Q;
  // K:2x1
  MatrixXd tmp(2, 2);
  tmp = H * Pk_pdct * H.transpose() + R;
  K = Pk_pdct * H.transpose() * tmp.inverse();
  //测量值z
  Z_meas << inPos.x, inPos.y; //真实值加随机噪声模拟测量值
  //估计值
  X_evlt = X_pdct + K * (Z_meas - H * X_pdct);
  //估计状态和真实状态的协方差矩阵，Pk
  Pk_evlt = (MatrixXd::Identity(4, 4) - K * H) * Pk_pdct;
  outPos.x = X_evlt(0, 0);
  outPos.y = X_evlt(1, 0);
  outPos.z = inPos.z;
}
/*************计算均值******************/
float GetMeanValue(const int num, const ROSPoi3d *que, const int &len,
                   const int &cur_index) {
  float sum(0);
  for (int i = 0; i < num; ++i) {
    sum += que[(cur_index - i + len) % len].z;
  }
  return sum / num;
}
/*************计算方差****************/
float GetVarianceValue(const int num, const ROSPoi3d *que, const int &len,
                       const int &cur_index) {
  float mean = GetMeanValue(num, que, len, cur_index);
  float sum(0), differ(0);
  for (int i = 0; i < num; ++i) {
    differ = que[(cur_index - i + len) % len].z - mean;
    sum += differ * differ;
  }
  return sum / num;
}
/****************计算均值和方差*******************/
void GetMeanAndVarianceValue(const int num, const ROSPoi3d *que, const int &len,
                             const int &cur_index, float &mean,
                             float &variance) {
  mean = GetMeanValue(num, que, len, cur_index);
  float sum(0), differ(0);
  for (int i = 0; i < num; ++i) {
    differ = que[(cur_index - i + len) % len].z - mean;
    sum += differ * differ;
  }
  variance = sum / num;
}
/**********剔除离群点***********/
bool IsOutliers(ROSPoi3d *que, const int len, const int cur_index) {
  static int point_count(0);
  static float mean(0), variance(0);
  if (point_count < 40) {
    ++point_count;
  } else {
    GetMeanAndVarianceValue(40, que, len, cur_index, mean, variance);
  }
  // printf("%f %f\n", mean, variance);
  if (variance > 0.2) {
    return true;
  } else {
    return false;
  }
}
#endif

// template <class Type> int _partition(Type data[], int first, int last) {
//   //尽量避免key是最大或最小值
//   int left(first), right(last);
//   int keyIndex = right; //把key值交换到数组右边末尾
//   while (left < right) {
//     while (left < right && data[left] <= data[keyIndex]) {
//       ++left;
//     }
//     while (left < right && data[right] >= data[keyIndex]) {
//       --right;
//     }
//     std::swap(data[left], data[right]);
//   }
//   std::swap(data[left], data[keyIndex]);
//   return left;
// }
// template <class Type> void QuickSort(Type data[], int first, int last) {
//   if (first >= last)
//     return;
//   int keyIndex = _partition(data, first, last);
//   QuickSort(data, first, keyIndex - 1);
//   QuickSort(data, keyIndex + 1, last);
// }
// // template <class Type>
// float MedianFilter(float data[], const int size)
// { //数组大小，也即取中值用到的点的数量，取为奇数
//   //对数组内的数据排序,由小到大
//   QuickSort<float>(data, 0, size - 1);
//   //返回中值
//   return data[size / 2];
// }
//*滑动平均滤波算法*/*均值滤波*/
// void Smooth(float data[], const int size, const int N)
// //数组首指针，数组大小,滑动平均滤波计算平均值时所取的点数(为偶数)
// {
//   float Sum1 = 0;
//   for (int j = 0; j < size; j++) {
//     if (j < N / 2) {
//       for (int k = 0; k < N; k++) {
//         Sum1 += data[j + k];
//       }
//       data[j] = Sum1 / N;
//     } else if (j < size - N / 2) {
//       for (int k = 0; k < N / 2; k++) {
//         Sum1 += (data[j + k] + data[j - k]);
//       }
//       data[j] = Sum1 / N;
//     } else {
//       for (int k = 0; k < size - j; k++) {
//         Sum1 += data[j + k];
//       }
//       for (int k = 0; k < (N - size + j); k++) {
//         Sum1 += data[j - k];
//       }
//       data[j] = Sum1 / N;
//     }
//     Sum1 = 0;
//   }
// }
