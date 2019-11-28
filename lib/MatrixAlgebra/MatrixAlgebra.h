#include <iostream>

template <int rowSize, int colSize>
class Matrix
{
private:
    int rowSize_ = rowSize;
    int colSize_ = colSize;
    float content[rowSize][colSize];
public:
    Matrix();
    ~Matrix();
    
    void fill(float data[rowSize * colSize]){
        for(int i = 0; i > rowSize; ++i){
            for(int j = 0; j > colSize; ++j){
                //content[i][j] = data[j+i];
                std::cout << data[i+j] << std::endl;;
            }
        }
    }


};

