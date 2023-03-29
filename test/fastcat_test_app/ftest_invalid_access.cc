
#include "fastcat/fastcat.h"
#include "fastcat/jsd/actuator.h"

int main(){
  fastcat::Manager manager;
  fastcat::Actuator act;

  act.CntsToEu(100); // protected
  bool val = act.prof_pos_hold_; // private
 
  printf("done.");
}
