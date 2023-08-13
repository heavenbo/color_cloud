#include <iostream>
#include <vector>
#include <cmath>

#include <tbb/parallel_for.h>

int main(int argc, char **argv)
{
    auto values = std::vector<double>(10000);
    
    tbb::parallel_for(tbb::blocked_range<int>(0, values.size(),1000),
                      [&](tbb::blocked_range<int> r)
                      {
                          for (int i = r.begin(); i < r.end(); ++i)
                          {
                              values[i] = 1;
                          }
                      });

    double total = 0;

    for (double value : values)
    {
        total += value;
    }

    std::cout << total << std::endl;

    return 0;
}