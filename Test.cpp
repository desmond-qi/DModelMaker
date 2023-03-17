#include <DModelMaker.hpp>

c_MMaker cMMaker7p2("bhr7p2_auto", 0.004, 1);

void main() {
    // add units            body name      inb name            mass    inerx    inery   inerz   if geom length  joints name                                             joints axis                             body to joint               inb to joint
    cMMaker7p2.fnvAddBase   ("midbody",     {0.0, 0.0, 0.94},   0.0001, {0.0001,  0.0001,  0.0001},       0.16                                                                                                                                                       );
    cMMaker7p2.fnvAddPin    ("uppbody",     "midbody",          19.500, {0.6100,  0.3800,  0.3000}, 1,    -0.5,   "waist_yaw",                                            {0, 0, 1},                            { 0.042,  0.000, -0.296},   { 0.000,  0.000,  0.000} );
    cMMaker7p2.fnvAddPin    ("larm",        "uppbody",          1.500,  {0.0430,  0.0440,  0.0023}, 1,    0.3,    "left_arm",                                             {0, 1, 0},                            {-0.019,  0.000,  0.229},   { 0.000,  0.272,  0.115} );
    cMMaker7p2.fnvAddPin    ("rarm",        "uppbody",          1.500,  {0.0430,  0.0440,  0.0023}, 1,    0.3,    "right_arm",                                            {0, 1, 0},                            {-0.019,  0.000,  0.229},   { 0.000, -0.272,  0.115} );
    cMMaker7p2.fnvAddBall   ("lthigh",      "midbody",          5.800,  {0.0670,  0.0630,  0.0140}, 1,    0.32,   "left_leg_1",   "left_leg_2",   "left_leg_3",           {0, 0, 1}, {1, 0, 0}, {0, 1, 0},      {-0.014, -0.015,  0.101},   { 0.000,  0.080,  0.000} );
    cMMaker7p2.fnvAddPin    ("lshank",      "lthigh",           1.800,  {0.0300,  0.0300,  0.0002}, 1,    0.32,   "left_leg_4",                                           {0, 1, 0},                            { 0.000,  0.000,  0.091},   {-0.014, -0.015, -0.219} );
    cMMaker7p2.fnvAddGimbal ("lankle",      "lshank",           1.150,  {0.0017,  0.0047,  0.0052}, 0,    0.0,    "left_l3eg_5",   "left_leg_6",                           {0, 1, 0}, {1, 0, 0},                 { 0.000,  0.000,  0.075},   { 0.000,  0.000, -0.229} );
    cMMaker7p2.fnvAddFoot   ("lfoot",       "lankle",           0.0001, {0.0001,  0.0001,  0.0001}, 1,    0.08,   "site_lforcesensor", "lforcesonsor", "ltorquesonsor",                                         { 0.135,  0.075,  0.010},   { 0.035,  0.020, -0.015} );
    cMMaker7p2.fnvAddBall   ("rthigh",      "midbody",          5.800,  {0.0670,  0.0630,  0.0140}, 1,    0.32,   "right_leg_1",  "right_leg_2",  "right_leg_3",          {0, 0, 1}, {1, 0, 0}, {0, 1, 0},      {-0.014,  0.015,  0.101},   { 0.000, -0.080,  0.000} );
    cMMaker7p2.fnvAddPin    ("rshank",      "rthigh",           1.800,  {0.0300,  0.0300,  0.0002}, 1,    0.32,   "right_leg_4",                                          {0, 1, 0},                            { 0.000,  0.000,  0.091},   {-0.014,  0.015, -0.219} );
    cMMaker7p2.fnvAddGimbal ("rankle",      "rshank",           1.150,  {0.0017,  0.0047,  0.0052}, 0,    0.0,    "right_leg_5",  "right_leg_6",                          {0, 1, 0}, {1, 0, 0},                 { 0.000,  0.000,  0.075},   { 0.000,  0.000, -0.229} );
    cMMaker7p2.fnvAddFoot   ("rfoot",       "rankle",           0.0001, {0.0001,  0.0001,  0.0001}, 1,    0.08,   "site_rforcesensor", "rforcesonsor", "rtorquesonsor",                                         { 0.135,  0.075,  0.010},   { 0.035, -0.020,  -0.015} );
    cMMaker7p2.fnvAddIMU    ("site_imu",    "uppbody",                                                                                                                                                                                      { 0.042,  0.000, -0.296} );
    
    // exclude contacts
    cMMaker7p2.fnvExContact("midbody", "uppbody");
    cMMaker7p2.fnvExContact("midbody", "larm");
    cMMaker7p2.fnvExContact("midbody", "rarm");
    cMMaker7p2.fnvExContact("midbody", "lthigh");
    cMMaker7p2.fnvExContact("midbody", "lshank");
    cMMaker7p2.fnvExContact("midbody", "lankle");
    cMMaker7p2.fnvExContact("midbody", "rthigh");
    cMMaker7p2.fnvExContact("midbody", "rshank");
    cMMaker7p2.fnvExContact("midbody", "rankle");
    cMMaker7p2.fnvExContact("uppbody", "larm");
    cMMaker7p2.fnvExContact("uppbody", "rarm");
    cMMaker7p2.fnvExContact("uppbody", "lthigh");
    cMMaker7p2.fnvExContact("uppbody", "lshank");
    cMMaker7p2.fnvExContact("uppbody", "lankle");
    cMMaker7p2.fnvExContact("uppbody", "rthigh");
    cMMaker7p2.fnvExContact("uppbody", "rshank");
    cMMaker7p2.fnvExContact("uppbody", "rankle");
    cMMaker7p2.fnvExContact("larm", "rarm");
    cMMaker7p2.fnvExContact("larm", "lthigh");
    cMMaker7p2.fnvExContact("larm", "lshank");
    cMMaker7p2.fnvExContact("larm", "lankle");
    cMMaker7p2.fnvExContact("larm", "rthigh");
    cMMaker7p2.fnvExContact("larm", "rshank");
    cMMaker7p2.fnvExContact("larm", "rankle");
    cMMaker7p2.fnvExContact("rarm", "lthigh");
    cMMaker7p2.fnvExContact("rarm", "lshank");
    cMMaker7p2.fnvExContact("rarm", "lankle");
    cMMaker7p2.fnvExContact("rarm", "rthigh");
    cMMaker7p2.fnvExContact("rarm", "rshank");
    cMMaker7p2.fnvExContact("rarm", "rankle");
    cMMaker7p2.fnvExContact("lthigh", "lshank");
    cMMaker7p2.fnvExContact("lthigh", "lankle");
    cMMaker7p2.fnvExContact("lthigh", "rthigh");
    cMMaker7p2.fnvExContact("lthigh", "rshank");
    cMMaker7p2.fnvExContact("lthigh", "rankle");
    cMMaker7p2.fnvExContact("lshank", "lankle");
    cMMaker7p2.fnvExContact("lshank", "rthigh");
    cMMaker7p2.fnvExContact("lshank", "rshank");
    cMMaker7p2.fnvExContact("lshank", "rankle");
    cMMaker7p2.fnvExContact("lankle", "rthigh");
    cMMaker7p2.fnvExContact("lankle", "rshank");
    cMMaker7p2.fnvExContact("lankle", "rankle");
    cMMaker7p2.fnvExContact("rthigh", "rshank");
    cMMaker7p2.fnvExContact("rthigh", "rankle");
    cMMaker7p2.fnvExContact("rshank", "rankle");



    cMMaker7p2.fnvWriteXML();
}