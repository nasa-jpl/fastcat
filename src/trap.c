#include "fastcat/trap.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>

#include "jsd/jsd_print.h"

#define SIGN(X) (((X) >= 0) - ((X) < 0))

// check for case where the initial or final velocity is such that
// the trap cannot reach the desired position without changing the sign
// of the velocity

// this function considers four cases where
// a) (vm>vi, vm>vf)
// b) (vm<vi, vm>vf)
// c) (vm<vi, vm<vf)
// d) (vm<vi, mv<vf)

// in case a) and d), the trap looks like a triangle in velocity space
// in case b),c) the trap is just a line in velocity space as acceleration
// is constant.

// in all cases, we want to make sure that the position moved during the
// acceleration phases from vi->vm and from vm->vf, does not exceed the
// total position delta (pf-pi).

// if this occurs, we are considering the limit case where the position moved
// during accelration phases = pf-pi.

// we enforce accel>0 always
// For case a)
// time_(from vi to vm) = t_im = (vm-vi)/accel
// time_(from vm to vf) = t_mf = (vm-vf)/accel
// distance travel from (vi to vm) d_im = 0.5*(vi+vm)*t_im;

// so we can combine the above equations to get:
// (pf-pi) = (vi+vm)(vm-vi)/(2acc) + (vf+vm)(vm-vf)/(2acc)
// or  2acc(pf-pi) = 2vm^2 -vi^2 -vf^2;
// or vm = +- sqrt( acc(pf-pi) +vi^2/2 +vf^2/2 )
// we then need to change the +- sign;

static bool profile_is_trapezoidal(double vel_max, double dist_total,
                                   double dist_acc, double dist_dec)
{
  if (vel_max * (dist_total - dist_acc - dist_dec) >= 0) {
    return true;
  } else {
    return false;
  }
}

static bool profile_is_a_positive_triangle(double vel_start, double vel_max,
                                           double vel_final)
{
  if (vel_start < vel_max && vel_max > vel_final) {
    return true;
  } else {
    return false;
  }
}

static bool profile_is_a_negative_triangle(double vel_start, double vel_max,
                                           double vel_final)
{
  if (vel_start > vel_max && vel_max < vel_final) {
    return true;
  } else {
    return false;
  }
}

static bool profile_is_a_positive_ramp(double vel_start, double vel_max,
                                       double vel_final)
{
  if (vel_start < vel_max && vel_max <= vel_final) {
    return true;
  } else {
    return false;
  }
}

static bool profile_is_a_negative_ramp(double vel_start, double vel_max,
                                       double vel_final)
{
  if (vel_start > vel_max && vel_max >= vel_final) {
    return true;
  } else {
    return false;
  }
}

/**
 * \brief Sets up math for creating a simple trapezoidal
 * position profile.
 * @param  self       trap main object
 * @param  t_init_sec initial absolute time in seconds
 * @param  pos_init   initial position for the profile
 * @param  pos_fini   final position for the profile
 * @param  vel_init   initial velocity for the profile
 * @param  vel_fini   final velocity for the profile
 * @param  vel_max    cruising speed for the profile
 * @param  acc        max acceleration/deceleration for the profile
 * @return            0 on success, -1 on failure
 */
int fastcat_trap_generate(fastcat_trap_t* self, double t_init_sec, double pos_init,
                  double pos_fini, double vel_init, double vel_fini,
                  double vel_max, double acc)
{
  /*
         Pos
       pf |
          |                        ------
          |                    ----
          |                  --
          |                 /
          |                /
          |               /
          |              /
          |             /
          |            /
          |           /
          |          /
          |-----------------------------------> time
       pi |___-----
          |
          |<--tim-->|<----tm----->|<-tmf->|
          |<-------------t_if-------------|

         Vel
          |
       vm |         _______________
          |        /|             |\
          |       / |  T O T A L  | \
          |      /  |             |  \
          |  acc/  D| I S T A N C |E  \dec
          |    /    |             |    \
          |   / dim |     dm      | dmf \
       vf |--/-------------------------------> time
          | /       |             |       |
       vi |/        |             |       |
          |<--tim-->|<----tm----->|<-tmf->|
          |<-------------t_if-------------|
  */
  double vi;    ///< initial velocity
  double vf;    ///< final velocity
  double vm;    ///< cruising ("max") velocity
  double d_if;  ///< total distance from initial to final
  double t_im;  ///< acceleration time from initial to max
  double t_mf;  ///< decleration time from max to final
  double d_im;  ///< acceleration distance from initial to max
  double d_mf;  ///< deceleration distance from max to final
  double pi;    ///< initial position
  double pf;    ///< final position
  double t_m;   ///< total time for constant velocity phase (cruising time)
  double d_m;   ///< distance travelled during constant velocity phase (cruising
                ///< distance)
  double eps = 1e-9;         ///< min value
  double vm_actual_squared;  ///< actual capped max velocity in order to not
                             ///< overshoot during a non-trapezoidal profile

  pi   = pos_init;
  pf   = pos_fini;
  d_if = pf - pi;   ///< total distance (including direction)
  vi   = vel_init;  ///< initial velocity
  vm   = SIGN(d_if) *
       fabs(vel_max);  // need max velocity to at least have the correct sign
  vf  = vel_fini;      ///< final velocity
  acc = fabs(acc);  ///< acceleration magnitude. direction is enforced elsewhere

  // Only accept accelerations higher than epsilon since the result would be
  // a very, very long, slow profile.
  if (acc < eps) {
    ERROR("acc (%.11f) < %.11f", acc, eps);
    return -1;
  }

  // Equations of motion for constant acceleration
  // time to accelerate to max velocity
  t_im = fabs(vm - vi) / acc;  // t_acc = delta_v/acc
  // time to decelerate from max velocity to final velocity
  t_mf = fabs(vf - vm) / acc;  // t_dec = delta_v/dec
  // distance travelled when starting at initial velocity and accelerating for
  // t_im
  d_im = (vi * t_im) + (0.5 * SIGN(vm - vi) * acc * t_im *
                        t_im);  // d_acc = vi*t_acc + 1/2*a*t_acc^2
  // distance travelled when starting at final velocity and decelerating for
  // t_mf
  d_mf = (vm * t_mf) + (0.5 * SIGN(vf - vm) * acc * t_mf *
                        t_mf);  // d_dec = vm*t_dec + 1/2*a*t_acc^2

  if (profile_is_trapezoidal(vm, d_if, d_im, d_mf)) {
    // the profile is a trap and we coast at vm (max velocity) for a while
    /*   _________
        /         \   or  \           /
       /           \       \_________/   */

    // delta position during max velocity phase (cruising distance)
    d_m =
        d_if - d_im -
        d_mf;  // total_distance - acceleration distance - deceleration distance

    // prevent too small of a max velocity
    if (fabs(vm) < eps) {
      vm = SIGN(vm) * eps;
    }

    // time during max velocity phase (cruising time)
    t_m = fabs(d_m / vm);  // time = distance / velocity

    if (!(t_m >= 0.0)) {
      ERROR("pf %5.2f, pi %5.2f, d_acc %5.2f, d_dec %5.2f", pf, pi, d_im,
                  d_mf);
    }

    // cruising time should always be positive.
    // should take out assertion and check return value to avoid losing
    // position, for example
    if (t_m < 0.0) {
      ERROR("Cruising time is negative. Failing trap. t_m: %f s", t_m);
      assert(t_m >= 0.0);
      return -1;
    }
  } else {
    /* Cases:  A       B       C       D
               /\     \  /     \        /
              /  \     \/       \      /
                  \              \    /
    */
    // if distance to accelerate and decelerate are greater than total distance
    // the profile is a triangle, and we never reach vm (max_velocity) (or are
    // at it for a 1/inf time)

    // we will limit the vm and vf phases to not overshoot
    if (profile_is_a_positive_triangle(vi, vm, vf))  // case A
    {
      /*    vm
            /|\        Positive triangular velocity profile
           / | \
          /d1|d2\ vf
      vi /   |

      vm^2 = vi^2 + 2*a*d1                                  (Equation of motion
      (eq.1)) vf^2 = vm^2 + 2*(-a)*d2                               (Equation of
      motion (eq.2)) d1 = (vm^2 - vi^2) / (2*a) (Solve eq.1 for d1) d2 = (vf^2 -
      vm^2) / (2*-a) = (vm^2 - vf^2) / (2*a)   (Solve eq.2 for d2) d1+d1 =
      ((vm^2 - vi^2) + (vm^2 - vf^2)) / (2*a)       (Add d1 and d2) 2*vm^2 =
      2*a*(d1+d2) + (vi^2 + vf^2)                  (Solve for vm) vm^2 =
      a*(d1+d2) + 0.5*(vi^2 + vf^2)                  (Solve for vm)
                                                      */
      vm_actual_squared =
          acc * d_if + 0.5 * (vi * vi + vf * vf);  // always positive

      // Doesn't work. vm_actual_squared will sometimes be less than eps with
      // low accelerations and distances, and that's ok! forcing it to be larger
      // makes the profile too long.
      // if (vm_actual_squared < eps) {
      //  vm_actual_squared = eps;
      //}

      vm = sqrt(vm_actual_squared);  //+-

      if (!((vi < vm) &&
            (vm > vf)))  // if this vm is not valid, choose other sol
      {
        vm = -vm;
        if (!((vi < vm) && (vm > vf))) {
          // if both sols not valid, set vm=vf, and modify vf;
          vm = vf;
        }
      }
    }

    if (profile_is_a_negative_triangle(vi, vm, vf))  // case B
    {
      /*      Negative triangular velocity profile
   vi \   |
       \d1|d2/ vf
        \ | /
         \|/
         vm
      vm^2 = vi^2 + 2*(-a)*d1                              (Equation of motion
   (eq.1)) vf^2 = vm^2 + 2*(a)*d2                               (Equation of
   motion (eq.2)) d1 = (vm^2 - vi^2) / (2*-a) = (vi^2 - vm^2) / (2*a)  (Solve
   eq.1 for d1) d2 = (vf^2 - vm^2) / (2*a)  (Solve for d2)           (Solve eq.2
   for d2) d1+d1 = ((vi^2 - vm^2) + (vf^2 - vm^2)) / (2*a)      (Add d1 and d2)
     -2*vm^2 = 2*a*(d1+d2) - (vi^2 + vf^2)                 (Solve for vm)
      vm^2 = -a*(d1+d2) + 0.5*(vi^2 + vf^2)                (Solve for vm)
                                                      */
      vm_actual_squared = -acc * d_if + 0.5 * (vi * vi + vf * vf);  //+-

      // if (vm_actual_squared < eps) {
      //  vm_actual_squared = eps;
      //}

      vm = sqrt(vm_actual_squared);

      if (!((vi > vm) &&
            (vm < vf)))  // if this vm is not valid, choose other sol
      {
        vm = -vm;
        if (!((vi > vm) && (vm < vf))) {
          // if both sols not valid, set vm=vf, and modify vf;
          vm = vf;
        }
      }
    }

    if (profile_is_a_negative_ramp(vi, vm, vf))  // case C
    {
      /*                Negative trapezoidal velocity profile
        \     or   \
         \.         \.
                      \   */
      // this means that vi > vm > vf, so vm is useless (as no cruizing phase)
      vm_actual_squared = vi * vi - 2 * acc * d_if;

      // if (vm_actual_squared < eps) {
      //  vm_actual_squared = eps;
      //}

      vf = sqrt(vm_actual_squared);

      if (!(vi > vf)) {
        vf = -vf;
      }

      vm = vf;
    }

    if (profile_is_a_positive_ramp(vi, vm, vf))  // case D
    {
      /*  Positive trapezoidal velocity profile
          .        ./
         /   or   /
        /        /                */
      // this means that vi<vm<vf, so vm is useless
      vm_actual_squared = vi * vi + 2 * acc * d_if;

      // if (vm_actual_squared < eps) {
      //  vm_actual_squared = eps;
      //}

      vf = sqrt(vm_actual_squared);

      if (!(vi < vf)) {
        vf = -vf;
      }

      vm = vf;
    }

    // triangle profile so no cruise distance or time
    t_m = 0.0;
    d_m = 0.0;

    t_im = fabs(vm - vi) / acc;
    t_mf = fabs(vm - vf) / acc;
    d_im = (vi * t_im) + (0.5 * SIGN(vm - vi) * acc * t_im * t_im);
    d_mf = (vm * t_mf) + (0.5 * SIGN(vf - vm) * acc * t_mf * t_mf);
  }

  // fill in the trajectory parameters
  self->pos_init = pi;
  self->pos_pre  = pi;
  self->pos_acc  = pi + d_im;
  self->pos_dec  = self->pos_acc + d_m;
  self->pos_fini = pf;

  self->t_init = t_init_sec;
  self->t_pre  = self->t_init;
  self->t_acc  = self->t_pre + t_im;
  self->t_dec  = self->t_acc + t_m;
  self->t_fini = self->t_dec + t_mf;

  self->vel_max = vm;
  self->acc_pre = acc;
  self->acc     = SIGN(vm - vi) * acc;
  self->dec     = SIGN(vf - vm) * acc;

  self->vel_init = vi;
  self->vel_fini = vf;
  self->vel_pre  = vi;
  self->vel_acc  = vm;

  self->mode = FASTCAT_TRAP_MODE_POS;

  return 0;
}

int fastcat_trap_update(fastcat_trap_t* self, double t, double* pos, double* vel)
{
  double dt;

  if (t < self->t_init) {
    // before the profile
    *pos = self->pos_init;
    *vel = self->vel_init;
    return 0;
  }

  if (t > self->t_fini) {
    // past the profile
    *pos = self->pos_fini;
    *vel = self->vel_fini;

    // if the profile has expired, report this back
    self->mode = FASTCAT_TRAP_MODE_IDLE;
    return 1;
  } else if (t < self->t_pre) {
    //
    dt   = t - self->t_init;
    *vel = self->vel_init + self->acc_pre * dt;
    *pos = self->pos_init + self->vel_init * dt + 0.5 * self->acc_pre * dt * dt;
  } else if (t < self->t_acc) {
    // accelerating (for dt time)
    dt   = t - self->t_pre;
    *vel = self->vel_pre + self->acc * dt;
    *pos = self->pos_pre + self->vel_pre * dt + 0.5 * self->acc * dt * dt;
  } else if (t >= self->t_dec) {
    // decelerating (for dt time)
    dt   = t - self->t_dec;
    *vel = self->vel_acc + self->dec * dt;
    *pos = self->pos_dec + self->vel_acc * dt + 0.5 * self->dec * dt * dt;
  } else {
    // at max velocity (for dt time)
    dt   = t - self->t_acc;
    *pos = self->pos_acc + self->vel_max * dt;
    *vel = self->vel_max;
  }

  return 0;
}

int fastcat_trap_generate_vel(fastcat_trap_t* self, double t_init_sec, double pos_init,
                      double vel_init, double vel_fini, double acc,
                      double max_time)
{
  double dt;

  self->pos_init = pos_init;
  self->t_init   = t_init_sec;
  self->vel_init = vel_init;
  self->vel_acc  = vel_fini;

  if (vel_fini > vel_init)
    self->acc = fabs(acc);
  else
    self->acc = -fabs(acc);

  if (max_time > 1e-9)  // there's a max time, after, decel to 0
  {
    self->vel_fini = 0;
  } else  // otherwise hold at vel_fini
  {
    self->vel_fini = vel_fini;
  }

  if (self->vel_fini > self->vel_acc)
    self->dec = fabs(acc);
  else
    self->dec = -fabs(acc);

  self->t_acc =
      self->t_init + fabs((self->vel_acc - self->vel_init) / self->acc);

  dt            = self->t_acc - self->t_init;
  self->pos_acc = pos_init + vel_init * dt + 0.5 * self->acc * dt * dt;

  self->t_dec = self->t_acc + max_time;

  dt            = self->t_dec - self->t_acc;
  self->pos_dec = self->pos_acc + self->vel_acc * dt;

  self->t_fini = self->t_dec + fabs(self->vel_acc / self->acc);
  dt           = self->t_fini - self->t_dec;
  self->pos_fini =
      self->pos_dec + self->vel_acc * dt + 0.5 * self->dec * dt * dt;

  self->mode = FASTCAT_TRAP_MODE_RATE;

  return 0;
}

int fastcat_trap_update_vel(fastcat_trap_t* self, double t, double* pos, double* vel)
{
  double dt;

  if (t < self->t_acc) {
    dt   = fmax(t - self->t_init, 0.001);
    *vel = self->vel_init + self->acc * dt;
    *pos = self->pos_init + self->vel_init * dt + 0.5 * self->acc * dt * dt;
  } else {
    if (fabs(self->vel_fini) <
        1e-9)  // if it's zero, we do a trapezoidal profile
    {
      if (t < self->t_dec)  // at constant vel (vel_acc)
      {
        dt   = t - self->t_acc;
        *vel = self->vel_acc;
        *pos = self->pos_acc + self->vel_acc * dt;
      } else if (t < self->t_fini)  // decelerating from vel_acc to zero
      {
        dt   = t - self->t_dec;
        *vel = self->vel_acc + self->dec * dt;
        *pos = self->pos_dec + self->vel_acc * dt + 0.5 * self->dec * dt * dt;
      } else  // if we're past t_fini, should be at zero velocity
      {
        *vel       = 0.0;
        *pos       = self->pos_fini;
        self->mode = FASTCAT_TRAP_MODE_IDLE;
        return 1;
      }
    } else  // no max time, spinning at v_acc until directed to change...
    {
      dt   = t - self->t_acc;
      *vel = self->vel_fini;  // keep at target velocity if no max time
      *pos = self->pos_acc + self->vel_acc * dt;
    }
  }

  return 0;
}

void fastcat_trap_print_fastcat_trap_values(fastcat_trap_t* self)
{
  MSG("Position: ini: %f, pre: %f, acc: %f, dec: %f, fin: %f", self->pos_init,
      self->pos_pre, self->pos_acc, self->pos_dec, self->pos_fini);
  MSG("Vel/Acc:  v_ini: %f, v_pre: %f, v_acc: %f, v_max: %f, v_fin: %f, acc: "
      "%f, acc_pre: %f, dec: %f",
      self->vel_init, self->vel_pre, self->vel_acc, self->vel_max,
      self->vel_fini, self->acc, self->acc_pre, self->dec);
  MSG("Time:     ini: %f, pre: %f, acc: %f, dec: %f, fin: %f", self->t_init,
      self->t_pre, self->t_acc, self->t_dec, self->t_fini);
}
