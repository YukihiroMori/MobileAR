#include "fileIO.hpp"

//デリミタで分割
vector<string> split(const string &str, char delim)
{
	vector<string> res;
	size_t current = 0, found;


	while((found = str.find_first_of(delim, current)) != string::npos){
	    res.push_back(string(str, current, found - current));
	    current = found + 1;
	}
	res.push_back(string(str, current, str.size() - current));
	return res;
}

//xyの表形式の読み込み　縦:点，横:カメラ
int readDataxy( vector< vector<Vector2d> > &data, string filename)
{
	ifstream ifs;
	ifs.open(filename.c_str());
	if(ifs.fail()){
		cout << "ファイルを開けません　：　" << filename << "\n";
		abort();
	}
	Vector2d x;
	vector<Vector2d> row_data;
	vector<string> str_data;
	string str;
	while(getline(ifs, str)){
		row_data.clear();

		str_data = split(str, ',');
		for(int i = 0; i < str_data.size() / 2 ; i++){
			x(0) = atof(str_data[i * 2].c_str());
			x(1) = atof(str_data[i * 2 + 1].c_str());
			row_data.push_back(x);
		}
		data.push_back(row_data);
	}
	ifs.close();
}
int readDataVec3d(vector<Vector3d> &data, string filename)
{
    ifstream ifs;
	ifs.open(filename.c_str());
	if(ifs.fail()){
		cout << filename << " is not found\n";
		abort();
	}

	Vector3d t;
	string str;
    int scanfN;
	while(getline(ifs, str)){
    	scanfN = sscanf(str.data(), "%lf %lf %lf", &t(0), &t(1), &t(2));

		data.push_back(t);
	}
	ifs.close();
    return scanfN;
}
int readDataMat3d(vector<Matrix3d> &data, string filename)
{
    ifstream ifs;
	ifs.open(filename.c_str());
	if(ifs.fail()){
		cout << filename << " is not found\n";
		abort();
	}

	Matrix3d R;
    int scanfN;
	string str;
	while(getline(ifs, str)){
    	scanfN = sscanf(str.data(), "%lf %lf %lf", &R(0, 0), &R(0, 1), &R(0, 2));
    	getline(ifs, str);
    	scanfN = sscanf(str.data(), "%lf %lf %lf", &R(1, 0), &R(1, 1), &R(1, 2));
    	getline(ifs, str);
    	scanfN = sscanf(str.data(), "%lf %lf %lf", &R(2, 0), &R(2, 1), &R(2, 2));

		data.push_back(R);
	}
	ifs.close();
    return scanfN;
}



void outDataPly(string filename, Vector3d data[], int dataN)
{
	ofstream ofs(filename.c_str());

	ofs << "ply" << endl;
	ofs << "format ascii 1.0" << endl;
	ofs << "comment " << endl;
	ofs << "element vertex " << dataN << endl;
	ofs << "property double x" << endl;
	ofs << "property double y" << endl;
	ofs << "property double z" << endl;
	ofs << "end_header" << endl;
	for(int i = 0; i < dataN; i++){
		ofs << data[i](0) << " ";
		ofs << data[i](1) << " ";
		ofs << data[i](2) << endl;
	}
	ofs.close();
}

void out_Mat3d(vector<Matrix3d> & data, string filename)
{
	int N = data.size();
	ofstream ofs(filename.c_str());

	for (int i = 0; i < N; i++){
		ofs << data[i] << endl;
	}

	ofs.close();
}

void out_Vec3d(vector<Vector3d> & data, string filename)
{
	int N = data.size();
	ofstream ofs(filename.c_str());

	for (int i = 0; i < N; i++){
		ofs << data[i].transpose() << endl;
	}

	ofs.close();
}
