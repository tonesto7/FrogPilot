#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3672539079200543080) {
   out_3672539079200543080[0] = delta_x[0] + nom_x[0];
   out_3672539079200543080[1] = delta_x[1] + nom_x[1];
   out_3672539079200543080[2] = delta_x[2] + nom_x[2];
   out_3672539079200543080[3] = delta_x[3] + nom_x[3];
   out_3672539079200543080[4] = delta_x[4] + nom_x[4];
   out_3672539079200543080[5] = delta_x[5] + nom_x[5];
   out_3672539079200543080[6] = delta_x[6] + nom_x[6];
   out_3672539079200543080[7] = delta_x[7] + nom_x[7];
   out_3672539079200543080[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1812608138244297507) {
   out_1812608138244297507[0] = -nom_x[0] + true_x[0];
   out_1812608138244297507[1] = -nom_x[1] + true_x[1];
   out_1812608138244297507[2] = -nom_x[2] + true_x[2];
   out_1812608138244297507[3] = -nom_x[3] + true_x[3];
   out_1812608138244297507[4] = -nom_x[4] + true_x[4];
   out_1812608138244297507[5] = -nom_x[5] + true_x[5];
   out_1812608138244297507[6] = -nom_x[6] + true_x[6];
   out_1812608138244297507[7] = -nom_x[7] + true_x[7];
   out_1812608138244297507[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_749084149301253579) {
   out_749084149301253579[0] = 1.0;
   out_749084149301253579[1] = 0;
   out_749084149301253579[2] = 0;
   out_749084149301253579[3] = 0;
   out_749084149301253579[4] = 0;
   out_749084149301253579[5] = 0;
   out_749084149301253579[6] = 0;
   out_749084149301253579[7] = 0;
   out_749084149301253579[8] = 0;
   out_749084149301253579[9] = 0;
   out_749084149301253579[10] = 1.0;
   out_749084149301253579[11] = 0;
   out_749084149301253579[12] = 0;
   out_749084149301253579[13] = 0;
   out_749084149301253579[14] = 0;
   out_749084149301253579[15] = 0;
   out_749084149301253579[16] = 0;
   out_749084149301253579[17] = 0;
   out_749084149301253579[18] = 0;
   out_749084149301253579[19] = 0;
   out_749084149301253579[20] = 1.0;
   out_749084149301253579[21] = 0;
   out_749084149301253579[22] = 0;
   out_749084149301253579[23] = 0;
   out_749084149301253579[24] = 0;
   out_749084149301253579[25] = 0;
   out_749084149301253579[26] = 0;
   out_749084149301253579[27] = 0;
   out_749084149301253579[28] = 0;
   out_749084149301253579[29] = 0;
   out_749084149301253579[30] = 1.0;
   out_749084149301253579[31] = 0;
   out_749084149301253579[32] = 0;
   out_749084149301253579[33] = 0;
   out_749084149301253579[34] = 0;
   out_749084149301253579[35] = 0;
   out_749084149301253579[36] = 0;
   out_749084149301253579[37] = 0;
   out_749084149301253579[38] = 0;
   out_749084149301253579[39] = 0;
   out_749084149301253579[40] = 1.0;
   out_749084149301253579[41] = 0;
   out_749084149301253579[42] = 0;
   out_749084149301253579[43] = 0;
   out_749084149301253579[44] = 0;
   out_749084149301253579[45] = 0;
   out_749084149301253579[46] = 0;
   out_749084149301253579[47] = 0;
   out_749084149301253579[48] = 0;
   out_749084149301253579[49] = 0;
   out_749084149301253579[50] = 1.0;
   out_749084149301253579[51] = 0;
   out_749084149301253579[52] = 0;
   out_749084149301253579[53] = 0;
   out_749084149301253579[54] = 0;
   out_749084149301253579[55] = 0;
   out_749084149301253579[56] = 0;
   out_749084149301253579[57] = 0;
   out_749084149301253579[58] = 0;
   out_749084149301253579[59] = 0;
   out_749084149301253579[60] = 1.0;
   out_749084149301253579[61] = 0;
   out_749084149301253579[62] = 0;
   out_749084149301253579[63] = 0;
   out_749084149301253579[64] = 0;
   out_749084149301253579[65] = 0;
   out_749084149301253579[66] = 0;
   out_749084149301253579[67] = 0;
   out_749084149301253579[68] = 0;
   out_749084149301253579[69] = 0;
   out_749084149301253579[70] = 1.0;
   out_749084149301253579[71] = 0;
   out_749084149301253579[72] = 0;
   out_749084149301253579[73] = 0;
   out_749084149301253579[74] = 0;
   out_749084149301253579[75] = 0;
   out_749084149301253579[76] = 0;
   out_749084149301253579[77] = 0;
   out_749084149301253579[78] = 0;
   out_749084149301253579[79] = 0;
   out_749084149301253579[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1433015691336091156) {
   out_1433015691336091156[0] = state[0];
   out_1433015691336091156[1] = state[1];
   out_1433015691336091156[2] = state[2];
   out_1433015691336091156[3] = state[3];
   out_1433015691336091156[4] = state[4];
   out_1433015691336091156[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1433015691336091156[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1433015691336091156[7] = state[7];
   out_1433015691336091156[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3763153453132433144) {
   out_3763153453132433144[0] = 1;
   out_3763153453132433144[1] = 0;
   out_3763153453132433144[2] = 0;
   out_3763153453132433144[3] = 0;
   out_3763153453132433144[4] = 0;
   out_3763153453132433144[5] = 0;
   out_3763153453132433144[6] = 0;
   out_3763153453132433144[7] = 0;
   out_3763153453132433144[8] = 0;
   out_3763153453132433144[9] = 0;
   out_3763153453132433144[10] = 1;
   out_3763153453132433144[11] = 0;
   out_3763153453132433144[12] = 0;
   out_3763153453132433144[13] = 0;
   out_3763153453132433144[14] = 0;
   out_3763153453132433144[15] = 0;
   out_3763153453132433144[16] = 0;
   out_3763153453132433144[17] = 0;
   out_3763153453132433144[18] = 0;
   out_3763153453132433144[19] = 0;
   out_3763153453132433144[20] = 1;
   out_3763153453132433144[21] = 0;
   out_3763153453132433144[22] = 0;
   out_3763153453132433144[23] = 0;
   out_3763153453132433144[24] = 0;
   out_3763153453132433144[25] = 0;
   out_3763153453132433144[26] = 0;
   out_3763153453132433144[27] = 0;
   out_3763153453132433144[28] = 0;
   out_3763153453132433144[29] = 0;
   out_3763153453132433144[30] = 1;
   out_3763153453132433144[31] = 0;
   out_3763153453132433144[32] = 0;
   out_3763153453132433144[33] = 0;
   out_3763153453132433144[34] = 0;
   out_3763153453132433144[35] = 0;
   out_3763153453132433144[36] = 0;
   out_3763153453132433144[37] = 0;
   out_3763153453132433144[38] = 0;
   out_3763153453132433144[39] = 0;
   out_3763153453132433144[40] = 1;
   out_3763153453132433144[41] = 0;
   out_3763153453132433144[42] = 0;
   out_3763153453132433144[43] = 0;
   out_3763153453132433144[44] = 0;
   out_3763153453132433144[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3763153453132433144[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3763153453132433144[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3763153453132433144[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3763153453132433144[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3763153453132433144[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3763153453132433144[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3763153453132433144[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3763153453132433144[53] = -9.8000000000000007*dt;
   out_3763153453132433144[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3763153453132433144[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3763153453132433144[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3763153453132433144[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3763153453132433144[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3763153453132433144[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3763153453132433144[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3763153453132433144[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3763153453132433144[62] = 0;
   out_3763153453132433144[63] = 0;
   out_3763153453132433144[64] = 0;
   out_3763153453132433144[65] = 0;
   out_3763153453132433144[66] = 0;
   out_3763153453132433144[67] = 0;
   out_3763153453132433144[68] = 0;
   out_3763153453132433144[69] = 0;
   out_3763153453132433144[70] = 1;
   out_3763153453132433144[71] = 0;
   out_3763153453132433144[72] = 0;
   out_3763153453132433144[73] = 0;
   out_3763153453132433144[74] = 0;
   out_3763153453132433144[75] = 0;
   out_3763153453132433144[76] = 0;
   out_3763153453132433144[77] = 0;
   out_3763153453132433144[78] = 0;
   out_3763153453132433144[79] = 0;
   out_3763153453132433144[80] = 1;
}
void h_25(double *state, double *unused, double *out_6602971945358739836) {
   out_6602971945358739836[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4947815980791958970) {
   out_4947815980791958970[0] = 0;
   out_4947815980791958970[1] = 0;
   out_4947815980791958970[2] = 0;
   out_4947815980791958970[3] = 0;
   out_4947815980791958970[4] = 0;
   out_4947815980791958970[5] = 0;
   out_4947815980791958970[6] = 1;
   out_4947815980791958970[7] = 0;
   out_4947815980791958970[8] = 0;
}
void h_24(double *state, double *unused, double *out_5086048052692273868) {
   out_5086048052692273868[0] = state[4];
   out_5086048052692273868[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3963750109950645302) {
   out_3963750109950645302[0] = 0;
   out_3963750109950645302[1] = 0;
   out_3963750109950645302[2] = 0;
   out_3963750109950645302[3] = 0;
   out_3963750109950645302[4] = 1;
   out_3963750109950645302[5] = 0;
   out_3963750109950645302[6] = 0;
   out_3963750109950645302[7] = 0;
   out_3963750109950645302[8] = 0;
   out_3963750109950645302[9] = 0;
   out_3963750109950645302[10] = 0;
   out_3963750109950645302[11] = 0;
   out_3963750109950645302[12] = 0;
   out_3963750109950645302[13] = 0;
   out_3963750109950645302[14] = 1;
   out_3963750109950645302[15] = 0;
   out_3963750109950645302[16] = 0;
   out_3963750109950645302[17] = 0;
}
void h_30(double *state, double *unused, double *out_4080506871530139339) {
   out_4080506871530139339[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5077154927935199040) {
   out_5077154927935199040[0] = 0;
   out_5077154927935199040[1] = 0;
   out_5077154927935199040[2] = 0;
   out_5077154927935199040[3] = 0;
   out_5077154927935199040[4] = 1;
   out_5077154927935199040[5] = 0;
   out_5077154927935199040[6] = 0;
   out_5077154927935199040[7] = 0;
   out_5077154927935199040[8] = 0;
}
void h_26(double *state, double *unused, double *out_9024369966293381714) {
   out_9024369966293381714[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8689319299666015194) {
   out_8689319299666015194[0] = 0;
   out_8689319299666015194[1] = 0;
   out_8689319299666015194[2] = 0;
   out_8689319299666015194[3] = 0;
   out_8689319299666015194[4] = 0;
   out_8689319299666015194[5] = 0;
   out_8689319299666015194[6] = 0;
   out_8689319299666015194[7] = 1;
   out_8689319299666015194[8] = 0;
}
void h_27(double *state, double *unused, double *out_8806986076906479095) {
   out_8806986076906479095[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7251918239735623951) {
   out_7251918239735623951[0] = 0;
   out_7251918239735623951[1] = 0;
   out_7251918239735623951[2] = 0;
   out_7251918239735623951[3] = 1;
   out_7251918239735623951[4] = 0;
   out_7251918239735623951[5] = 0;
   out_7251918239735623951[6] = 0;
   out_7251918239735623951[7] = 0;
   out_7251918239735623951[8] = 0;
}
void h_29(double *state, double *unused, double *out_9082180139190984984) {
   out_9082180139190984984[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8965280966605174984) {
   out_8965280966605174984[0] = 0;
   out_8965280966605174984[1] = 1;
   out_8965280966605174984[2] = 0;
   out_8965280966605174984[3] = 0;
   out_8965280966605174984[4] = 0;
   out_8965280966605174984[5] = 0;
   out_8965280966605174984[6] = 0;
   out_8965280966605174984[7] = 0;
   out_8965280966605174984[8] = 0;
}
void h_28(double *state, double *unused, double *out_6202580756097781444) {
   out_6202580756097781444[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4399064090034846058) {
   out_4399064090034846058[0] = 1;
   out_4399064090034846058[1] = 0;
   out_4399064090034846058[2] = 0;
   out_4399064090034846058[3] = 0;
   out_4399064090034846058[4] = 0;
   out_4399064090034846058[5] = 0;
   out_4399064090034846058[6] = 0;
   out_4399064090034846058[7] = 0;
   out_4399064090034846058[8] = 0;
}
void h_31(double *state, double *unused, double *out_3962499477596762595) {
   out_3962499477596762595[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4917170018914998542) {
   out_4917170018914998542[0] = 0;
   out_4917170018914998542[1] = 0;
   out_4917170018914998542[2] = 0;
   out_4917170018914998542[3] = 0;
   out_4917170018914998542[4] = 0;
   out_4917170018914998542[5] = 0;
   out_4917170018914998542[6] = 0;
   out_4917170018914998542[7] = 0;
   out_4917170018914998542[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_3672539079200543080) {
  err_fun(nom_x, delta_x, out_3672539079200543080);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1812608138244297507) {
  inv_err_fun(nom_x, true_x, out_1812608138244297507);
}
void car_H_mod_fun(double *state, double *out_749084149301253579) {
  H_mod_fun(state, out_749084149301253579);
}
void car_f_fun(double *state, double dt, double *out_1433015691336091156) {
  f_fun(state,  dt, out_1433015691336091156);
}
void car_F_fun(double *state, double dt, double *out_3763153453132433144) {
  F_fun(state,  dt, out_3763153453132433144);
}
void car_h_25(double *state, double *unused, double *out_6602971945358739836) {
  h_25(state, unused, out_6602971945358739836);
}
void car_H_25(double *state, double *unused, double *out_4947815980791958970) {
  H_25(state, unused, out_4947815980791958970);
}
void car_h_24(double *state, double *unused, double *out_5086048052692273868) {
  h_24(state, unused, out_5086048052692273868);
}
void car_H_24(double *state, double *unused, double *out_3963750109950645302) {
  H_24(state, unused, out_3963750109950645302);
}
void car_h_30(double *state, double *unused, double *out_4080506871530139339) {
  h_30(state, unused, out_4080506871530139339);
}
void car_H_30(double *state, double *unused, double *out_5077154927935199040) {
  H_30(state, unused, out_5077154927935199040);
}
void car_h_26(double *state, double *unused, double *out_9024369966293381714) {
  h_26(state, unused, out_9024369966293381714);
}
void car_H_26(double *state, double *unused, double *out_8689319299666015194) {
  H_26(state, unused, out_8689319299666015194);
}
void car_h_27(double *state, double *unused, double *out_8806986076906479095) {
  h_27(state, unused, out_8806986076906479095);
}
void car_H_27(double *state, double *unused, double *out_7251918239735623951) {
  H_27(state, unused, out_7251918239735623951);
}
void car_h_29(double *state, double *unused, double *out_9082180139190984984) {
  h_29(state, unused, out_9082180139190984984);
}
void car_H_29(double *state, double *unused, double *out_8965280966605174984) {
  H_29(state, unused, out_8965280966605174984);
}
void car_h_28(double *state, double *unused, double *out_6202580756097781444) {
  h_28(state, unused, out_6202580756097781444);
}
void car_H_28(double *state, double *unused, double *out_4399064090034846058) {
  H_28(state, unused, out_4399064090034846058);
}
void car_h_31(double *state, double *unused, double *out_3962499477596762595) {
  h_31(state, unused, out_3962499477596762595);
}
void car_H_31(double *state, double *unused, double *out_4917170018914998542) {
  H_31(state, unused, out_4917170018914998542);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
