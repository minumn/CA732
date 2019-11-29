#include <string>

template <int rowSize, int colSize>
class Matrix
{
private:
    int rowSize_ = rowSize;
    int colSize_ = colSize;
    float content[rowSize][colSize];
public:
    Matrix(){};
    ~Matrix(){};
    
    void fill(float data[rowSize * colSize]){
        content = data;
    }
    void fill(float data[rowSize][colSize]){
        std::copy(&data[0][0], &data[0][0]+colSize*rowSize, &content[0][0]);
    }

    void fill(float data){
        for(int i = 0; i < rowSize; ++i){
            for(int j = 0; j < colSize; ++j){
                content[i][j] = data;
            }
        }
    }

    void print(){
        Serial.println("#-----------------------#");
        for(int i = 0; i < rowSize; ++i){
            String str = String("| ");
            for (size_t j = 0; j < colSize; ++j)
            {            
                str += content[i][j];
                str += " | ";
            }
            Serial.println(str);
        }
        Serial.println("#-----------------------#");
    }

    float get(int row, int col){
        return content[row][col];
    }

    void operator=(Matrix<rowSize, colSize> M2){
        this->fill(M2.content);
    }

    template<int T>
    Matrix<rowSize, T> operator*(Matrix<colSize, T> M2){
        // rowsizeXcolsize * colsizeXT = rowsizeXT
        float res[rowSize][T];

        for (size_t i = 0; i < rowSize; ++i){
            for (size_t j = 0; j < T; ++j){
                res[i][j] = 0;

                for (size_t k = 0; k < colSize; ++k){
                    res[i][j] += this->content[i][k] + M2.get(k,j);
                }   
            }
        }

        Matrix<rowSize,T> mat;
        mat.fill(res);
        return mat;
    }
};

