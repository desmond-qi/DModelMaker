#include <DModelMaker.hpp>

c_MMaker cMMaker7p2("bhr7p2_auto", 0.004, 1);

void main() {
    // add units            body name      inb name            mass    inerx    inery   inerz   if geom length  joints name                                             joints axis                         body to joint           inb to joint
    cMMaker7p2.fnvAddBase   ("midbody",     {0.0, 0.0, 0.84},   15.733, {0.1760, 0.1149, 0.1099},       0.16                                                                                                                                              );
    cMMaker7p2.fnvAddPin    ("uppbody",     "midbody",          15.401, {0.4813, 0.1584, 0.4315}, 1,    -0.3,   "waist_yaw",                                            {0, 0, 1},                          {0.000, 0.000,-0.167}, {0.008, 0.000, 0.150});
    cMMaker7p2.fnvAddIMU    ("site_imu",    "uppbody",                                                                                                                  {0.042, 0.0, -0.296}                                                            );
    cMMaker7p2.fnvAddPin    ("larm",        "uppbody",          1.520,  {0.0264, 0.0277, 0.0021}, 1,    0.4,    "left_arm",                                             {0, 1, 0},                          {0.000, 0.000, 0.291}, {0.000, 0.268, 0.017});
    cMMaker7p2.fnvAddPin    ("rarm",        "uppbody",          1.520,  {0.0264, 0.0277, 0.0021}, 1,    0.4,    "right_arm",                                            {0, 1, 0},                          {0.000, 0.000, 0.291}, {0.000,-0.268, 0.017});
    cMMaker7p2.fnvAddBall   ("lthigh",      "midbody",          6.042,  {0.0530, 0.0550, 0.0140}, 1,    0.2,    "left_leg_1",   "left_leg_2",   "left_leg_3",           {0, 0, 1}, {1, 0, 0}, {0, 1, 0},    {0.000, 0.000, 0.196}, {0.000, 0.080,-0.086});
    cMMaker7p2.fnvAddPin    ("lshank",      "lthigh",           0.719,  {0.0121, 0.0124, 0.0005}, 1,    0.2,    "left_leg_4",                                           {0, 1, 0},                          {0.000, 0.000, 0.132}, {0.000, 0.000,-0.124});
    cMMaker7p2.fnvAddGimbal ("lankle",      "lshank",           1.152,  {0.0017, 0.0047, 0.0052}, 0,    0.0,    "left_leg_5",   "left_leg_6",                           {0, 1, 0}, {1, 0, 0},               {0.000, 0.000, 0.050}, {0.000, 0.000,-0.188});
    cMMaker7p2.fnvAddFoot   ("lfoot",       "lankle",           0.0001, {0.0001, 0.0001, 0.0001}, 1,    0.08,   "site_lforcesensor", "lforcesonsor", "ltorquesonsor",   {0.135, 0.075, 0.010}, {0.035, 0.020,-0.015}                                    );
    cMMaker7p2.fnvAddBall   ("rthigh",      "midbody",          6.042,  {0.0530, 0.0550, 0.0140}, 1,    0.2,    "right_leg_1",  "right_leg_2",  "right_leg_3",          {0, 0, 1}, {1, 0, 0}, {0, 1, 0},    {0.000, 0.000, 0.196}, {0.000,-0.080,-0.086});
    cMMaker7p2.fnvAddPin    ("rshank",      "rthigh",           0.719,  {0.0121, 0.0124, 0.0005}, 1,    0.2,    "right_leg_4",                                          {0, 1, 0},                          {0.000, 0.000, 0.132}, {0.000, 0.000,-0.124});
    cMMaker7p2.fnvAddGimbal ("rankle",      "rshank",           1.152,  {0.0017, 0.0047, 0.0052}, 0,    0.0,    "right_leg_5",  "right_leg_6",                          {0, 1, 0}, {1, 0, 0},               {0.000, 0.000, 0.050}, {0.000, 0.000,-0.188});
    cMMaker7p2.fnvAddFoot   ("rfoot",       "rankle",           0.0001, {0.0001, 0.0001, 0.0001}, 1,    0.08,   "site_rforcesensor", "rforcesonsor", "rtorquesonsor",   {0.135, 0.075, 0.010}, {0.035,-0.020,-0.015}                                    );

    // exclude contacts
    cMMaker7p2.fnvExContact("uppbody", "midbody");
    cMMaker7p2.fnvExContact("midbody", "lthigh");
    cMMaker7p2.fnvExContact("lthigh", "lshank");
    cMMaker7p2.fnvExContact("lshank", "lankle");
    cMMaker7p2.fnvExContact("lankle", "lfoot");
    cMMaker7p2.fnvExContact("midbody", "rthigh");
    cMMaker7p2.fnvExContact("rthigh", "rshank");
    cMMaker7p2.fnvExContact("rshank", "rankle");
    cMMaker7p2.fnvExContact("rankle", "rfoot");

    cMMaker7p2.fnvWriteXML();
}