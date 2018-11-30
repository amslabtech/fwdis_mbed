#include "steering.h"

Steering::Steering(void)
{
  encoder_fr.initialize();
  encoder_fl.initialize();
  encoder_rl.initialize();
  encoder_rr.initialize();
}
