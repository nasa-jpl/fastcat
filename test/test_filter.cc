#include "fastcat/fastcat.h"
#include "fastcat/fastcat_devices/filter.h"

int main(int argc, char* argv[])
{
  (void)argc;
  (void)argv;

  // 2nd order Butterworth, Wn=0.5
  // std::vector<double> A = {1.0, 0.0, 0.1715729};
  // std::vector<double> B = {0.2928932, 0.5857864, 0.2928932};

  // 2nd order Butterworth, Wn=0.05
  // std::vector<double> A = {1.0, -1.778631, 0.800803};
  // std::vector<double> B = {0.005542717, 0.0110854434, 0.0055427172};

  // Moving Average
  std::vector<double>      A = {0};
  std::vector<double>      B = {0.25, 0.25, 0.25, 0.25};
  fastcat::DigitalABFilter filter(A, B);

  filter.ApplyFilter(1);
  filter.ApplyFilter(1);
  filter.ApplyFilter(1);
  filter.ApplyFilter(1);
  filter.ApplyFilter(1);
  filter.ApplyFilter(1);
  filter.ApplyFilter(1);
  filter.ApplyFilter(1);
  filter.ApplyFilter(1);
  filter.ApplyFilter(1);
  filter.ApplyFilter(1);

  fastcat::MovingAverageFilter mafilt(4);

  MSG_DEBUG("ma = %lf", mafilt.ApplyFilter(1));
  MSG_DEBUG("ma = %lf", mafilt.ApplyFilter(1));
  MSG_DEBUG("ma = %lf", mafilt.ApplyFilter(1));
  MSG_DEBUG("ma = %lf", mafilt.ApplyFilter(1));
  MSG_DEBUG("ma = %lf", mafilt.ApplyFilter(1));
  MSG_DEBUG("ma = %lf", mafilt.ApplyFilter(1));
  MSG_DEBUG("ma = %lf", mafilt.ApplyFilter(1));
  MSG_DEBUG("ma = %lf", mafilt.ApplyFilter(1));
  MSG_DEBUG("ma = %lf", mafilt.ApplyFilter(1));
  MSG_DEBUG("ma = %lf", mafilt.ApplyFilter(1));
  MSG_DEBUG("ma = %lf", mafilt.ApplyFilter(1));

  return 0;
}
