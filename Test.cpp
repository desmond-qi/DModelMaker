#include <DModelMaker.hpp>

#define BHR7P2
// #define BHRGAO

c_MMaker cMMakerBHR("bhrGao", 0.004, 0, 1);

void main() {
    // --------------------------------------------------------- for BHR7p2 ----------------------------------------------------------------------------
    #ifdef BHR7P2
    // add units            body name      inb name            mass    inerx    inery   inerz   if geom length  joints name                                             joints axis                             body to joint               inb to joint
    cMMakerBHR.fnvAddBase   ("midbody",     {0.0, 0.0, 0.72},   0.0001, {0.0001,  0.0001,  0.0001},       0.16                                                                                                                                                       );
    cMMakerBHR.fnvAddPin    ("uppbody",     "midbody",          19.500, {0.6100,  0.3800,  0.3000}, 1,    -0.5,   "waist_yaw",                                            {0, 0, 1},                            { 0.042,  0.000, -0.296},   { 0.000,  0.000,  0.000} );
    cMMakerBHR.fnvAddPin    ("larm",        "uppbody",          1.500,  {0.0430,  0.0440,  0.0023}, 1,    0.3,    "left_arm",                                             {0, 1, 0},                            {-0.019,  0.000,  0.229},   { 0.000,  0.272,  0.115} );
    cMMakerBHR.fnvAddPin    ("rarm",        "uppbody",          1.500,  {0.0430,  0.0440,  0.0023}, 1,    0.3,    "right_arm",                                            {0, 1, 0},                            {-0.019,  0.000,  0.229},   { 0.000, -0.272,  0.115} );
    cMMakerBHR.fnvAddBall   ("lthigh",      "midbody",          5.800,  {0.0670,  0.0630,  0.0140}, 1,    0.32,   "left_leg_1",   "left_leg_2",   "left_leg_3",           {0, 0, 1}, {1, 0, 0}, {0, 1, 0},      {-0.014, -0.015,  0.101},   { 0.000,  0.080,  0.000} );
    cMMakerBHR.fnvAddPin    ("lshank",      "lthigh",           1.800,  {0.0300,  0.0300,  0.0002}, 1,    0.32,   "left_leg_4",                                           {0, 1, 0},                            { 0.000,  0.000,  0.091},   {-0.014, -0.015, -0.219} );
    cMMakerBHR.fnvAddGimbal ("lankle",      "lshank",           1.150,  {0.0017,  0.0047,  0.0052}, 0,    0.0,    "left_l3eg_5",   "left_leg_6",                          {0, 1, 0}, {1, 0, 0},                 { 0.000,  0.000,  0.075},   { 0.000,  0.000, -0.229} );
    cMMakerBHR.fnvAddFoot   ("lfoot",       "lankle",           0.0001, {0.0001,  0.0001,  0.0001}, 1,    0.08,   "site_lforcesensor", "lforcesonsor", "ltorquesonsor",                                         { 0.135,  0.075,  0.010},   { 0.035,  0.020, -0.015} );
    cMMakerBHR.fnvAddBall   ("rthigh",      "midbody",          5.800,  {0.0670,  0.0630,  0.0140}, 1,    0.32,   "right_leg_1",  "right_leg_2",  "right_leg_3",          {0, 0, 1}, {1, 0, 0}, {0, 1, 0},      {-0.014,  0.015,  0.101},   { 0.000, -0.080,  0.000} );
    cMMakerBHR.fnvAddPin    ("rshank",      "rthigh",           1.800,  {0.0300,  0.0300,  0.0002}, 1,    0.32,   "right_leg_4",                                          {0, 1, 0},                            { 0.000,  0.000,  0.091},   {-0.014,  0.015, -0.219} );
    cMMakerBHR.fnvAddGimbal ("rankle",      "rshank",           1.150,  {0.0017,  0.0047,  0.0052}, 0,    0.0,    "right_leg_5",  "right_leg_6",                          {0, 1, 0}, {1, 0, 0},                 { 0.000,  0.000,  0.075},   { 0.000,  0.000, -0.229} );
    cMMakerBHR.fnvAddFoot   ("rfoot",       "rankle",           0.0001, {0.0001,  0.0001,  0.0001}, 1,    0.08,   "site_rforcesensor", "rforcesonsor", "rtorquesonsor",                                         { 0.135,  0.075,  0.010},   { 0.035, -0.020,  -0.015} );
    cMMakerBHR.fnvAddIMU    ("site_imu",    "uppbody",                                                                                                                                                                                      { 0.042,  0.000, -0.296} );
    
    // exclude contacts
    cMMakerBHR.fnvExContact("midbody", "uppbody");
    cMMakerBHR.fnvExContact("midbody", "larm");
    cMMakerBHR.fnvExContact("midbody", "rarm");
    cMMakerBHR.fnvExContact("midbody", "lthigh");
    cMMakerBHR.fnvExContact("midbody", "lshank");
    cMMakerBHR.fnvExContact("midbody", "lankle");
    cMMakerBHR.fnvExContact("midbody", "rthigh");
    cMMakerBHR.fnvExContact("midbody", "rshank");
    cMMakerBHR.fnvExContact("midbody", "rankle");
    cMMakerBHR.fnvExContact("uppbody", "larm");
    cMMakerBHR.fnvExContact("uppbody", "rarm");
    cMMakerBHR.fnvExContact("uppbody", "lthigh");
    cMMakerBHR.fnvExContact("uppbody", "lshank");
    cMMakerBHR.fnvExContact("uppbody", "lankle");
    cMMakerBHR.fnvExContact("uppbody", "rthigh");
    cMMakerBHR.fnvExContact("uppbody", "rshank");
    cMMakerBHR.fnvExContact("uppbody", "rankle");
    cMMakerBHR.fnvExContact("larm", "rarm");
    cMMakerBHR.fnvExContact("larm", "lthigh");
    cMMakerBHR.fnvExContact("larm", "lshank");
    cMMakerBHR.fnvExContact("larm", "lankle");
    cMMakerBHR.fnvExContact("larm", "rthigh");
    cMMakerBHR.fnvExContact("larm", "rshank");
    cMMakerBHR.fnvExContact("larm", "rankle");
    cMMakerBHR.fnvExContact("rarm", "lthigh");
    cMMakerBHR.fnvExContact("rarm", "lshank");
    cMMakerBHR.fnvExContact("rarm", "lankle");
    cMMakerBHR.fnvExContact("rarm", "rthigh");
    cMMakerBHR.fnvExContact("rarm", "rshank");
    cMMakerBHR.fnvExContact("rarm", "rankle");
    cMMakerBHR.fnvExContact("lthigh", "lshank");
    cMMakerBHR.fnvExContact("lthigh", "lankle");
    cMMakerBHR.fnvExContact("lthigh", "rthigh");
    cMMakerBHR.fnvExContact("lthigh", "rshank");
    cMMakerBHR.fnvExContact("lthigh", "rankle");
    cMMakerBHR.fnvExContact("lshank", "lankle");
    cMMakerBHR.fnvExContact("lshank", "rthigh");
    cMMakerBHR.fnvExContact("lshank", "rshank");
    cMMakerBHR.fnvExContact("lshank", "rankle");
    cMMakerBHR.fnvExContact("lankle", "rthigh");
    cMMakerBHR.fnvExContact("lankle", "rshank");
    cMMakerBHR.fnvExContact("lankle", "rankle");
    cMMakerBHR.fnvExContact("rthigh", "rshank");
    cMMakerBHR.fnvExContact("rthigh", "rankle");
    cMMakerBHR.fnvExContact("rshank", "rankle");
    #endif

    // --------------------------------------------------------- for BHRGao ----------------------------------------------------------------------------
    #ifdef BHRGAO
    // add units            body name      inb name            mass           inerx          inery         inerz          if geom       length      joints name                                             joints axis                            body to joint                            inb to joint
    cMMakerBHR.fnvAddBase   ("midbody",     {0.0, 0.0, 0.72},   1.098500e+01, {5.729103e-01, 3.337385e-01, 6.890181e-01},               0.16                                                                                                                                                                                                     );
    cMMakerBHR.fnvAddPin    ("uppbody",     "midbody",          2.314000e+01, {3.609189e-01, 3.344450e-01, 2.276843e-01}, 1,            -0.5,       "waist_yaw",                                            {0, 0, 1},                            { 5.255984e-02,  0.0000000000, -1.837984e-01},   { 9.183309e-03,  0.0000000000,  2.908694e-03} );
    cMMakerBHR.fnvAddPin    ("larm",        "uppbody",          7.988000e+00, {1.830000e-01, 1.832000e-01, 1.160000e-01}, 1,            0.3,        "left_arm",                                             {0, 1, 0},                            {-2.203950e-03, -3.376923e-03,  8.368240e-02},   { 1.255984e-02,  2.138000e-01,  7.550164e-02} );
    cMMakerBHR.fnvAddPin    ("rarm",        "uppbody",          7.988000e+00, {1.830000e-01, 1.832000e-01, 1.160000e-01}, 1,            0.3,        "right_arm",                                            {0, 1, 0},                            {-2.203950e-03,  3.376923e-03,  8.368240e-02},   { 1.255984e-02, -2.138000e-01,  7.550164e-02} );
    cMMakerBHR.fnvAddPin    ("lhip",        "midbody",          4.316000e+00, {6.271148e-02, 5.317312e-02, 4.203784e-02}, 0,            0.0,        "left_leg_1",                                           {0, 0, 1},                            { 9.300000e-04,  3.540000e-03, -1.540000e-02},   {-3.081669e-02,  9.000000e-02, -1.994913e-01} );
    cMMakerBHR.fnvAddGimbal ("lthigh",      "lhip",             5.573000e+00, {9.683885e-02, 9.913514e-02, 1.881028e-02}, 1,            0.32,       "left_leg_3",   "left_leg_2",                           {0, 1, 0}, {1, 0, 0},                 { 1.137156e-03, -3.784052e-03,  6.478608e-02},   { 9.300000e-04, -3.960000e-03, -1.540000e-02} );
    cMMakerBHR.fnvAddPin    ("lshank",      "lthigh",           1.982000e+00, {9.008190e-02, 8.849630e-02, 6.453392e-03}, 1,            0.32,       "left_leg_4",                                           {0, 1, 0},                            { 4.530000e-02,  4.140000e-03,  1.744000e-01},   { 5.113716e-02, -3.784052e-03, -2.657139e-01} );
    cMMakerBHR.fnvAddGimbal ("lankle",      "lshank",           1.405000e+00, {2.942070e-03, 6.114560e-03, 5.133870e-03}, 0,            0.0,        "left_l3eg_5",   "left_leg_6",                          {0, 1, 0}, {1, 0, 0},                 { 4.040000e-03, -1.350000e-03,  3.166000e-02},   {-4.700000e-03,  4.140000e-03, -1.556000e-01} );
    cMMakerBHR.fnvAddFoot   ("lfoot",       "lankle",           1.166800e+00, {2.068313e-03, 5.036445e-03, 5.344496e-03}, 1,            0.08,       "site_lforcesensor", "lforcesonsor", "ltorquesonsor",                                         { 0.135,  0.075,  0.010},                        { 0.035,  0.020, -0.015} );
    cMMakerBHR.fnvAddPin    ("rhip",        "midbody",          4.316000e+00, {6.271148e-02, 5.317312e-02, 4.203784e-02}, 0,            0.0,        "right_leg_1",                                          {0, 0, 1},                            { 9.300000e-04, -3.540000e-03, -1.540000e-02},   {-3.081669e-02, -9.000000e-02, -1.994913e-01} );
    cMMakerBHR.fnvAddGimbal ("rthigh",      "rhip",             5.573000e+00, {9.683885e-02, 9.913514e-02, 1.881028e-02}, 1,            0.32,       "right_leg_3",   "right_leg_2",                         {0, 1, 0}, {1, 0, 0},                 { 1.137156e-03,  3.784052e-03,  6.478608e-02},   { 9.300000e-04,  3.960000e-03, -1.540000e-02} );
    cMMakerBHR.fnvAddPin    ("rshank",      "rthigh",           1.982000e+00, {9.008190e-02, 8.849630e-02, 6.453392e-03}, 1,            0.32,       "right_leg_4",                                          {0, 1, 0},                            { 4.530000e-02, -4.140000e-03,  1.744000e-01},   { 5.113716e-02,  3.784052e-03, -2.657139e-01} );
    cMMakerBHR.fnvAddGimbal ("rankle",      "rshank",           1.405000e+00, {2.942070e-03, 6.114560e-03, 5.133870e-03}, 0,            0.0,        "right_l3eg_5",   "right_leg_6",                        {0, 1, 0}, {1, 0, 0},                 { 4.040000e-03,  1.350000e-03,  3.166000e-02},   {-4.700000e-03, -4.140000e-03, -1.556000e-01} );
    cMMakerBHR.fnvAddFoot   ("rfoot",       "rankle",           1.166800e+00, {2.068313e-03, 5.036445e-03, 5.344496e-03}, 1,            0.08,       "site_rforcesensor", "rforcesonsor", "rtorquesonsor",                                         { 0.135,  0.075,  0.010},                        { 0.035,  0.020, -0.015} );
    cMMakerBHR.fnvAddIMU    ("site_imu",    "uppbody",                                                                                                                                                                                            { 0.042,  0.000, -0.296} );
    
    // exclude contacts
    cMMakerBHR.fnvExContact("midbody", "uppbody");
    cMMakerBHR.fnvExContact("midbody", "larm");
    cMMakerBHR.fnvExContact("midbody", "rarm");
    cMMakerBHR.fnvExContact("midbody", "lhip");
    cMMakerBHR.fnvExContact("midbody", "lthigh");
    cMMakerBHR.fnvExContact("midbody", "lshank");
    cMMakerBHR.fnvExContact("midbody", "lankle");
    cMMakerBHR.fnvExContact("midbody", "rhip");
    cMMakerBHR.fnvExContact("midbody", "rthigh");
    cMMakerBHR.fnvExContact("midbody", "rshank");
    cMMakerBHR.fnvExContact("midbody", "rankle");
    cMMakerBHR.fnvExContact("uppbody", "larm");
    cMMakerBHR.fnvExContact("uppbody", "rarm");
    cMMakerBHR.fnvExContact("uppbody", "lthigh");
    cMMakerBHR.fnvExContact("uppbody", "lshank");
    cMMakerBHR.fnvExContact("uppbody", "lankle");
    cMMakerBHR.fnvExContact("uppbody", "rthigh");
    cMMakerBHR.fnvExContact("uppbody", "rshank");
    cMMakerBHR.fnvExContact("uppbody", "rankle");
    cMMakerBHR.fnvExContact("larm", "rarm");
    cMMakerBHR.fnvExContact("larm", "lhip");
    cMMakerBHR.fnvExContact("larm", "lthigh");
    cMMakerBHR.fnvExContact("larm", "lshank");
    cMMakerBHR.fnvExContact("larm", "lankle");
    cMMakerBHR.fnvExContact("larm", "rhip");
    cMMakerBHR.fnvExContact("larm", "rthigh");
    cMMakerBHR.fnvExContact("larm", "rshank");
    cMMakerBHR.fnvExContact("larm", "rankle");
    cMMakerBHR.fnvExContact("rarm", "lhip");
    cMMakerBHR.fnvExContact("rarm", "lthigh");
    cMMakerBHR.fnvExContact("rarm", "lshank");
    cMMakerBHR.fnvExContact("rarm", "lankle");
    cMMakerBHR.fnvExContact("rarm", "rhip");
    cMMakerBHR.fnvExContact("rarm", "rthigh");
    cMMakerBHR.fnvExContact("rarm", "rshank");
    cMMakerBHR.fnvExContact("rarm", "rankle");
    cMMakerBHR.fnvExContact("lhip", "lthigh");
    cMMakerBHR.fnvExContact("lhip", "lshank");
    cMMakerBHR.fnvExContact("lhip", "lankle");
    cMMakerBHR.fnvExContact("lhip", "rhip");
    cMMakerBHR.fnvExContact("lhip", "rthigh");
    cMMakerBHR.fnvExContact("lhip", "rshank");
    cMMakerBHR.fnvExContact("lhip", "rankle");
    cMMakerBHR.fnvExContact("lthigh", "lshank");
    cMMakerBHR.fnvExContact("lthigh", "lankle");
    cMMakerBHR.fnvExContact("lthigh", "rthigh");
    cMMakerBHR.fnvExContact("lthigh", "rshank");
    cMMakerBHR.fnvExContact("lthigh", "rankle");
    cMMakerBHR.fnvExContact("lshank", "lankle");
    cMMakerBHR.fnvExContact("lshank", "rthigh");
    cMMakerBHR.fnvExContact("lshank", "rshank");
    cMMakerBHR.fnvExContact("lshank", "rankle");
    cMMakerBHR.fnvExContact("lankle", "rthigh");
    cMMakerBHR.fnvExContact("lankle", "rshank");
    cMMakerBHR.fnvExContact("lankle", "rankle");
    cMMakerBHR.fnvExContact("rhip", "rthigh");
    cMMakerBHR.fnvExContact("rhip", "rshank");
    cMMakerBHR.fnvExContact("rhip", "rankle");
    cMMakerBHR.fnvExContact("rthigh", "rshank");
    cMMakerBHR.fnvExContact("rthigh", "rankle");
    cMMakerBHR.fnvExContact("rshank", "rankle");
    #endif


    cMMakerBHR.fnvWriteXML();

    cMMakerBHR.fnvDisp();
}