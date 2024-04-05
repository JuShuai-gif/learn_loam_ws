#ifndef _TIC_TOC_H_
#define _TIC_TOC_H_

#include <ctime>
#include <cstdlib>
#include <chrono>

class TicToc
{
private:
    std::chrono::time_point<std::chrono::system_clock> start,end;

public:
    TicToc(){
        tic();
    }

    void tic(){
        start = std::chrono::system_clock::now();
    }

    double toc(){
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_senonds = end - start;
        return elapsed_senonds.count() * 1000;
    }
};

#endif // _TIC_TOC_H_