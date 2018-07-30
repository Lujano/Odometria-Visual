#include "sort.cpp"
using namespace std; 

int main()
{
	int arrayNum[50],t; 
	cout<<"Enter Number of Elements:";
	cin>>t;
	for(int y = 0; y<t; y++)
	{
		cin>>arrayNum[y];
	}
	_introsort(arrayNum, t);
	for(int x = 0; x<t; x++)
	{
		std::cout << x << ":" << arrayNum[x] << std::endl;
	}
	cin.get();
	return 0;
} 