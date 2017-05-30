#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

//カンマ分割
vector<string> split(const string &str, char delim);
//xyの表形式の読み込み　縦:点，横:カメラ
int readDataxy( vector< vector<Vector2d> > &data, string filename);
//Vector3dを読み込む
int readDataVec3d(vector<Vector3d> &data, string filename);
//Matrix3dを読み込む
int readDataMat3d(vector<Matrix3d> &data, string filename);

void outDataPly(string filename, Vector3d data[], int dataN);
void out_Mat3d(vector<Matrix3d> & data, string filename);
void out_Vec3d(vector<Vector3d> & data, string filename);
