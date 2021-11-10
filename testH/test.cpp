#include <fstream>
#include <iostream>
using namespace std;

int main(){
	ifstream ifp;
	ofstream ofp;

	ofp.open("test.bin", ios::binary);

	for(int i = 0; i < 10; i++){
		cout<<(char*)&i<<'\n';
		ofp.write((char*)&i, sizeof(int));
	}
	ofp.close();

	ifp.open("test.bin", ios::binary);
	for(int i = 0; i < 10; i++){
		int temp;
		ifp.read((char*)&temp, sizeof(int));
		cout<<temp<<' ';
	}
	cout<<'\n';

	return 0;
}
