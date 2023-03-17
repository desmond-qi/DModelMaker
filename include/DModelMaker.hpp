// DModelMaker.hpp
// 20230314 dcc <3120195094@bit.edu.cn>
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <array>

#define __MaxStrLen 60
#define __MaxBodyNum 50
#define __MaxJointNum 50
#define __Tab                   fprintf(this->m_file, "\t");
#define __BodyTab(rank)         { __Tab __Tab for(int i = 0; i < rank; i++) __Tab }
#define __Etr                   fprintf(this->m_file, "\n");
#define __Start(content)        fprintf(this->m_file, "<"); fprintf(this->m_file, content); fprintf(this->m_file, ">"); __Etr
#define __StartMujoco(content, input) \
            fprintf(this->m_file, "<"); fprintf(this->m_file, content, input); fprintf(this->m_file, ">"); __Etr
#define __StartBody(content, input1, input2, input3, input4) \
            fprintf(this->m_file, "<"); fprintf(this->m_file, content, input1, input2, input3, input4); fprintf(this->m_file, ">"); __Etr
#define __End(content)          fprintf(this->m_file, "</"); fprintf(this->m_file, content); fprintf(this->m_file, ">"); __Etr
#define __Line(content)         fprintf(this->m_file, "<"); fprintf(this->m_file, content); fprintf(this->m_file, "/>"); __Etr
#define __LineIn1(content, input) \
            fprintf(this->m_file, "<"); fprintf(this->m_file, content, input); fprintf(this->m_file, "/>"); __Etr
#define __LineIn2(content, input1, input2) \
            fprintf(this->m_file, "<"); fprintf(this->m_file, content, input1, input2); fprintf(this->m_file, "/>"); __Etr
#define __LineIn3(content, input1, input2, input3) \
            fprintf(this->m_file, "<"); fprintf(this->m_file, content, input1, input2, input3); fprintf(this->m_file, "/>"); __Etr
#define __LineIn4(content, input1, input2, input3, input4) \
            fprintf(this->m_file, "<"); fprintf(this->m_file, content, input1, input2, input3, input4); fprintf(this->m_file, "/>"); __Etr
#define __LineIn7(content, input1, input2, input3, input4, input5, input6, input7) \
            fprintf(this->m_file, "<"); fprintf(this->m_file, content, input1, input2, input3, input4, input5, input6, input7); fprintf(this->m_file, "/>"); __Etr

template<int N>
using doubleN = std::array<double, N>;

enum en_JointType {
    pin = 1, gim, bal, fs
};

struct st_JointBody {
    double Mass; // mass of the body
    double Iner[3]; // rotational inertia
    double i2b[3]; // inbord to body
    double b2j[3]; // body to joint
    double i2j[3]; // inbord to joint
    double axis[3][3]; // the axis direction of the joint
    double len; // the length of the diplayed body
    char jointName[3][__MaxStrLen];
    char bodyName[__MaxStrLen];
    char geomName[__MaxStrLen];
    char meshPath[__MaxStrLen];
    char meshName[__MaxStrLen];
    int rank; // the linkage level of the body, which relates to the number of tabs
    int BodytNo; // the number of the body
    int jointType; // the type of the joint
    int hasIMU; // if has IMU intergrated
    int ifGeom; // if use geom to contact
    double i2IMU[3]; // inbord to IMU
    char IMUName[__MaxStrLen];
};

class c_MMaker {
public:
    c_MMaker(const char * cptModelName, double dTimeStep, int nIfGravity) {
        strcpy_s(this->m_cptModelName, cptModelName);
        this->m_dTimeStep = dTimeStep;
        this->m_nGravityFlag = nIfGravity;
        this->fnvOpenFile();
        this->fnvWriteDefault();
        this->m_nBodyNum = 0;
        this->m_nJointNum = 0;
        this->m_nExContactNum = 0;
    }
    
    ~c_MMaker() {
        this->fnvCloseFile();
    }

    void fnvAddBase(
        char *      cptBodyName, 
        doubleN<3>  dOriPos,
        double      dMass, 
        doubleN<3>  dIner, 
        double      dLinkLength) {
        auto & Info = this->m_stBodysInfo[this->m_nBodyNum];
        strcpy(Info.jointName[0], "root"); // copy the joint name
        strcpy(Info.bodyName, cptBodyName); // copy the body name
        strcpy(Info.geomName, cptBodyName); // copy the body name
        strcat(Info.geomName, "_geom"); // arrange the geom name
        for(int i = 0; i < 3; i++) {
            Info.i2b[i] = dOriPos[i]; // original position of the floating nase
            Info.Iner[i] = dIner[i]; // rotation inertia
        }
        Info.Mass = dMass; // mass
        Info.len = dLinkLength; // linkage length 
        Info.rank = 0;
        Info.BodytNo = this->m_nBodyNum; // body number
    }

    void fnvAddPin(
        char *      cptBodyName, 
        char *      cptInbName,
        double      dMass, 
        doubleN<3>  dIner, 
        int         nIfGeom,
        double      dLinkLength,
        char *      cptJointName, 
        doubleN<3>  dAxis,  
        doubleN<3>  dBody2Joint,
        doubleN<3>  dInb2Joint) {
        this->m_nBodyNum++; // add the body number
        auto & Info = this->m_stBodysInfo[this->m_nBodyNum];
        Info.ifGeom = nIfGeom;
        strcpy(Info.jointName[0], cptJointName); // copy the joint name to bodyinfo
        strcpy(this->m_cptJointsList[this->m_nJointNum], cptJointName); // copy the joint name to jointlist
        this->m_nJointNum++; // add the joint number
        strcpy(Info.bodyName, cptBodyName); // copy the body name
        strcpy(Info.geomName, cptBodyName); // copy the body name
        strcat(Info.geomName, "_geom"); // arrange the geom name
        for(int i = 0; i < 3; i++) {
            Info.axis[0][i] = dAxis[i]; // joint axis
            Info.i2j[i] = dInb2Joint[i]; // inbord to joint
            Info.b2j[i] = dBody2Joint[i]; // body to joint
            Info.i2b[i] = dInb2Joint[i] - dBody2Joint[i]; // inbord to body
            Info.Iner[i] = dIner[i]; // rotation inertia
        }
       Info.Mass = dMass; // mass
       Info.len = dLinkLength; // linkage length 
        for(int i = 0; i < __MaxBodyNum; i++) {
            if(strcmp(cptInbName, this->m_stBodysInfo[i].bodyName) == 0) {
                Info.rank = this->m_stBodysInfo[i].rank + 1; 
                break;
            }// find inbord and get rank
            if(i >= this->m_nBodyNum) { // didn't find the same inbord body name
                _STD cout << "Wrong inbord body name for body: " << Info.bodyName << _STD endl;
                break;
            }
        }
        Info.BodytNo = this->m_nBodyNum; // body number
        Info.jointType = pin; // joint type
    }

    void fnvAddGimbal(
        char *      cptBodyName, 
        char *      cptInbName,
        double      dMass, 
        doubleN<3>  dIner, 
        int         nIfGeom,
        double      dLinkLength,
        char *      cptJointName1, 
        char *      cptJointName2, 
        doubleN<3>  dAxis1, 
        doubleN<3>  dAxis2, 
        doubleN<3>  dBody2Joint,
        doubleN<3>  dInb2Joint) {
        this->m_nBodyNum++; // add the body number
        auto & Info = this->m_stBodysInfo[this->m_nBodyNum];
        Info.ifGeom = nIfGeom;
        strcpy(Info.jointName[0], cptJointName1); // copy the joint name to bodyinfo
        strcpy(this->m_cptJointsList[this->m_nJointNum], cptJointName1); // copy the joint name to jointlist
        this->m_nJointNum++; // add the joint number
        strcpy(Info.jointName[1], cptJointName2); // copy the joint name to bodyinfo
        strcpy(this->m_cptJointsList[this->m_nJointNum], cptJointName2); // copy the joint name to jointlist
        this->m_nJointNum++; // add the joint number
        strcpy(Info.bodyName, cptBodyName); // copy the body name
        strcpy(Info.geomName, cptBodyName); // copy the body name
        strcat(Info.geomName, "_geom"); // arrange the geom name
        for(int i = 0; i < 3; i++) {
            Info.axis[0][i] = dAxis1[i]; // joint axis
            Info.axis[1][i] = dAxis2[i]; // joint axis
            Info.i2j[i] = dInb2Joint[i]; // inbord to joint
            Info.b2j[i] = dBody2Joint[i]; // body to joint
            Info.i2b[i] = dInb2Joint[i] - dBody2Joint[i]; // inbord to body
            Info.Iner[i] = dIner[i]; // rotation inertia
        }
       Info.Mass = dMass; // mass
       Info.len = dLinkLength; // linkage length 
        for(int i = 0; i < __MaxBodyNum; i++) {
            if(strcmp(cptInbName, this->m_stBodysInfo[i].bodyName) == 0) {
                Info.rank = this->m_stBodysInfo[i].rank + 1; 
                break;
            }// find inbord and get rank
            if(i >= this->m_nBodyNum) { // didn't find the same inbord body name
                _STD cout << "Wrong inbord body name for body: " << Info.bodyName << _STD endl;
                break;
            }
        }
        Info.BodytNo = this->m_nBodyNum; // body number
        Info.jointType = gim; // joint type
    }

    void fnvAddBall(
        char *      cptBodyName, 
        char *      cptInbName,
        double      dMass, 
        doubleN<3>  dIner, 
        int         nIfGeom,
        double      dLinkLength,
        char *      cptJointName1, 
        char *      cptJointName2, 
        char *      cptJointName3, 
        doubleN<3>  dAxis1, 
        doubleN<3>  dAxis2, 
        doubleN<3>  dAxis3, 
        doubleN<3>  dBody2Joint,
        doubleN<3>  dInb2Joint) {
        this->m_nBodyNum++; // add the body number
        auto & Info = this->m_stBodysInfo[this->m_nBodyNum];
        Info.ifGeom = nIfGeom;
        strcpy(Info.jointName[0], cptJointName1); // copy the joint name to bodyinfo
        strcpy(this->m_cptJointsList[this->m_nJointNum], cptJointName1); // copy the joint name to jointlist
        this->m_nJointNum++; // add the joint number
        strcpy(Info.jointName[1], cptJointName2); // copy the joint name to bodyinfo
        strcpy(this->m_cptJointsList[this->m_nJointNum], cptJointName2); // copy the joint name to jointlist
        this->m_nJointNum++; // add the joint number
        strcpy(Info.jointName[2], cptJointName3); // copy the joint name to bodyinfo
        strcpy(this->m_cptJointsList[this->m_nJointNum], cptJointName3); // copy the joint name to jointlist
        this->m_nJointNum++; // add the joint number
        strcpy(Info.bodyName, cptBodyName); // copy the body name
        strcpy(Info.geomName, cptBodyName); // copy the body name
        strcat(Info.geomName, "_geom"); // arrange the geom name
        for(int i = 0; i < 3; i++) {
            Info.axis[0][i] = dAxis1[i]; // joint axis
            Info.axis[1][i] = dAxis2[i]; // joint axis
            Info.axis[2][i] = dAxis3[i]; // joint axis
            Info.i2j[i] = dInb2Joint[i]; // inbord to joint
            Info.b2j[i] = dBody2Joint[i]; // body to joint
            Info.i2b[i] = dInb2Joint[i] - dBody2Joint[i]; // inbord to body
            Info.Iner[i] = dIner[i]; // rotation inertia
        }
       Info.Mass = dMass; // mass
       Info.len = dLinkLength; // linkage length 
        for(int i = 0; i < __MaxBodyNum; i++) {
            if(strcmp(cptInbName, this->m_stBodysInfo[i].bodyName) == 0) {
                Info.rank = this->m_stBodysInfo[i].rank + 1; 
                break;
            }// find inbord and get rank
            if(i >= this->m_nBodyNum) { // didn't find the same inbord body name
                _STD cout << "Wrong inbord body name for body: " << Info.bodyName << _STD endl;
                break;
            }
        }
        Info.BodytNo = this->m_nBodyNum; // body number
        Info.jointType = bal; // joint type
    }

    void fnvAddFoot(
        char *      cptBodyName, 
        char *      cptInbName,
        double      dMass, 
        doubleN<3>  dIner, 
        int         nIfGeom,
        double      dForceSensorHeight,
        char *      cptForceSensorName,
        char *      cptForceName,
        char *      cptTorqueName,
        doubleN<3>  dSize, 
        doubleN<3>  dInb2Body) {
        this->m_nBodyNum++; // add the body number
        auto & Info = this->m_stBodysInfo[this->m_nBodyNum];
        Info.ifGeom = nIfGeom;
        strcpy(Info.jointName[0], cptForceSensorName); // copy the force sensor name
        strcpy(Info.jointName[1], cptForceName); // copy the force handle name
        strcpy(Info.jointName[2], cptTorqueName); // copy the torque handle name
        strcpy(Info.bodyName, cptBodyName); // copy the body name
        strcpy(Info.geomName, cptBodyName); // copy the body name
        strcat(Info.geomName, "_geom"); // arrange the geom name
        for(int i = 0; i < 3; i++) {
            Info.b2j[i] = dSize[i]; // b2j is used for recording foot size
            Info.i2b[i] = dInb2Body[i]; // inbord to body
            Info.Iner[i] = dIner[i]; // rotation inertia
        }
       Info.Mass = dMass; // mass
       Info.len = dForceSensorHeight; // ankle height 
        for(int i = 0; i < __MaxBodyNum; i++) {
            if(strcmp(cptInbName, this->m_stBodysInfo[i].bodyName) == 0) {
                Info.rank = this->m_stBodysInfo[i].rank + 1; 
                break;
            }// find inbord and get rank
            if(i >= this->m_nBodyNum) { // didn't find the same inbord body name
                _STD cout << "Wrong inbord body name for body: " << Info.bodyName << _STD endl;
                break;
            }
        }
        Info.BodytNo = this->m_nBodyNum; // body number
        Info.jointType = fs; // joint type
    }

    void fnvAddIMU(
        char *      cptIMUName, 
        char *      cptInbName,
        doubleN<3>  dInb2IMU) {
        for(int i = 0; i < __MaxBodyNum; i++) {
            if(strcmp(cptInbName, this->m_stBodysInfo[i].bodyName) == 0) {
                this->m_stBodysInfo[i].hasIMU = 1;
                strcpy_s(this->m_stBodysInfo[i].IMUName, cptIMUName);
                for(int j = 0; j < 3; j++) this->m_stBodysInfo[i].i2IMU[j] = dInb2IMU[j];
                break;
            }// find inbord and get rank
            if(i >= this->m_nBodyNum) { // didn't find the same inbord body name
                _STD cout << "Wrong inbord body name for IMU: " << cptIMUName << _STD endl;
                break;
            }
        }
    }

    void fnvExContact(char * cptBody1, char * cptBody2) {
        strcpy_s(this->m_cptExContactList[this->m_nExContactNum][0], cptBody1);
        strcpy_s(this->m_cptExContactList[this->m_nExContactNum][1], cptBody2);        
        this->m_nExContactNum++;
    }

    void fnvWriteXML() {
        this->m_nBodyNum++; // add the counting number of floating base body
        this->fnvWriteAsset();
        this->fnvWriteWorld();
        this->fnvWriteSettings();
    }



private:
    int m_nJointNum;
    int m_nBodyNum;
    char m_cptModelName[__MaxStrLen];
    double m_dTimeStep;
    int m_nGravityFlag;
    FILE * m_file;
    st_JointBody m_stBodysInfo[__MaxBodyNum];
    char m_cptJointsList[__MaxJointNum][__MaxStrLen];
    char m_cptExContactList[100][2][__MaxStrLen];
    int m_nExContactNum;
    void fnvOpenFile() {
        char wchFileName[__MaxStrLen];
        strcpy_s(wchFileName, m_cptModelName);
        strcat_s(wchFileName, ".xml");
        this->m_file = fopen(wchFileName, "w"); 
    }

    void fnvWriteDefault() {
        __StartMujoco("mujoco model = \"%s\"", this->m_cptModelName) __Etr 

        __Tab __Line("compiler inertiafromgeom = \"false\" angle = \"radian\""); __Etr 

        __Tab __Start("default") 
        __Tab __Tab __Line("joint limited = \"true\" stiffness = \"1500\" damping = \"10\" armature = \"2.5\"");
        __Tab __Tab __Line("geom condim = \"4\" material = \"matgeom\"");
        __Tab __Tab __Line("motor ctrlrange = \"-314.0 314.0\" ctrllimited = \"true\"");
        __Tab __Tab __Line("position kp = \"15\"");
        __Tab __End("default") __Etr

        __Tab fprintf(this->m_file, "<option timestep = \"%lf\" ", this->m_dTimeStep); fprintf(this->m_file, "apirate = \"144\" iterations = \"50\"/>"); __Etr
        __Tab __Line("option impratio = \"0.1\"") 
        __Tab __Line("option tolerance = \"1e-10\" solver = \"Newton\"")
        __Tab 
        if(this->m_nGravityFlag) {
            __Line("option jacobian = \"dense\" cone = \"pyramidal\" gravity = \"0.0 0.0 -9.8\"") __Etr
        }
        else {
            __Line("option jacobian = \"dense\" cone = \"pyramidal\" gravity = \"0.0 0.0 0.0\"") __Etr
        }
        
        __Tab __Line("size nconmax = \"50\" njmax = \"200\" nstack = \"10000\"") __Etr

        __Tab __Start("visual")
        __Tab __Tab __Line("map force = \"0.5\" zfar = \"30\"")
        __Tab __Tab __Line("rgba haze = \"0.1 0.1 0.1 0\"")
        __Tab __Tab __Line("quality shadowsize = \"4096\"")
        __Tab __Tab __Line("global offwidth = \"1600\" offheight = \"1600\"")
        __Tab __End("visual") __Etr
    }
    
    void fnvWriteAsset() {
        __Tab __Start("asset")
        __Tab __Tab __Line("texture type=\"skybox\" builtin=\"gradient\" rgb1=\"0.5 0.8 1.0\" rgb2=\"0.4 0.6 0.7\" width=\"512\" height=\"512\"")
        __Tab __Tab __Line("texture name=\"texplane\" type=\"2d\" builtin=\"checker\" rgb1=\".2 .3 .4\" rgb2=\".1 0.15 0.2\" width=\"512\" height=\"512\" mark=\"cross\" markrgb=\".8 .8 .8\"")
        __Tab __Tab __Line("texture name=\"texgeom\" type=\"cube\" builtin=\"flat\" mark=\"cross\" width=\"127\" height=\"1278\" rgb1=\"0.8 0.6 0.4\" rgb2=\"0.8 0.6 0.4\" markrgb=\"1 1 1\" random=\"0.01\"")
        __Tab __Tab __Line("material name=\"matplane\" reflectance=\"0.3\" texture=\"texplane\" texrepeat=\"1 1\" rgba =\"0.9 0.9 0.9 1\"")
        __Tab __Tab __Line("material name=\"matgeom\" texture=\"texgeom\" texuniform=\"true\" rgba=\"0.8 0.6 .4 1\"")
        __Tab __Tab __Line("material name=\"robots\" shininess=\"0.5\" reflectance=\"0.5\"")
        __Tab __End("asset") __Etr
    }

    void fnvWriteBody(int nBodyNum) {
        auto & Info = this->m_stBodysInfo[nBodyNum], & _Info = this->m_stBodysInfo[nBodyNum - 1];
        int nRankDown = (_Info.rank - Info.rank);
        for(int i = 0; i < nRankDown + 1; i++) { // write end body
            int rank = _Info.rank - i;
            __BodyTab(rank) __End("body") 
        }
        __BodyTab(Info.rank) __StartBody("body name = \"%s\" pos = \"%lf %lf %lf\"", Info.bodyName, Info.i2b[0], Info.i2b[1], Info.i2b[2]) // write body
        __Tab __BodyTab(Info.rank) __LineIn4("inertial pos = \"0 0 0\" mass = \"%lf\" diaginertia = \"%lf %lf %lf\"", Info.Mass, Info.Iner[0], Info.Iner[1], Info.Iner[2]) // write dynamics
        if(Info.jointType == fs) { // if the body is foot
            if(Info.ifGeom) {
                __Tab __BodyTab(Info.rank) __LineIn4("geom name = \"%s\" type = \"box\" size = \"%lf %lf %lf\" rgba = \"0.14 0.16 0.16 1\" material = \"robots\"", Info.bodyName, Info.b2j[0], Info.b2j[1], Info.b2j[2]) // write geom
            }
            __Tab __BodyTab(Info.rank) __LineIn4("site name = \"%s\" pos = \"%lf %lf %lf\"", Info.jointName[0], -Info.i2b[0], -Info.i2b[1], Info.len)
        }
        else { // if the body contains ordinary joints
            for(int i = 0; i < Info.jointType; i++) { // write joints
                __Tab __BodyTab(Info.rank) __LineIn7("joint name = \"%s\" type=\"hinge\" pos = \"%lf %lf %lf\" axis = \"%lf %lf %lf\" range = \"-90 90\"", Info.jointName[i], Info.b2j[0], Info.b2j[1], Info.b2j[2], Info.axis[i][0], Info.axis[i][1], Info.axis[i][2])
            }
            if(Info.ifGeom) {
                __Tab __BodyTab(Info.rank) __LineIn3("geom name = \"%s\" type = \"capsule\" fromto = \"0 0 %lf 0 0 %lf\" size = \"0.04\" rgba = \"0.14 0.16 0.16 1\" material = \"robots\"", Info.geomName, Info.b2j[2], Info.b2j[2] - Info.len) // write geom
            }
        }
        if(Info.hasIMU) { // if has IMU on this body
            __Tab __BodyTab(Info.rank) __LineIn4("site name = \"%s\" pos = \"%lf %lf %lf\"", Info.IMUName, Info.i2IMU[0], Info.i2IMU[1], Info.i2IMU[2])
        }
    }

    void fnvWriteWorld() {
        // write default world body
        __Tab __Start("worldbody")
        __Tab __Tab __Line("geom name = \"floor\" pos = \"0 0 0\" size = \"0 0 .25\" type = \"plane\" material = \"matplane\" condim = \"3\" friction = \"1.3 0.005 0.0001\"")
        __Tab __Tab __Line("light directional = \"true\" diffuse = \".5 .5 .5\" specular = \"0.3 0.3 0.3\" pos = \"0 0 5\" dir = \"-0.25 0 -1\" castshadow = \"false\"")
        __Tab __Tab __Line("light mode = \"targetbodycom\" target = \"midbody\" directional = \"true\" diffuse = \".7 .7 .7\" specular = \"0.3 0.3 0.3\" pos = \"0 0 4.0\" dir = \"0 0 -1\" castshadow = \"false\"") __Etr
        { // write base body
        auto & Info = this->m_stBodysInfo[0]; 
        __BodyTab(0) __StartBody("body name = \"%s\" pos = \"%lf %lf %lf\"", Info.bodyName, Info.i2b[0], Info.i2b[1], Info.i2b[2])
        __Tab __BodyTab(0) __LineIn1("freejoint name = \"%s\"", Info.jointName)
        __Tab __BodyTab(0) __LineIn4("inertial pos = \"0 0 0\" mass = \"%lf\" diaginertia = \"%lf %lf %lf\"", Info.Mass, Info.Iner[0], Info.Iner[1], Info.Iner[2])
        __Tab __BodyTab(0) __LineIn3("geom name = \"%s\" type = \"capsule\" fromto = \"0 %lf 0 0 %lf 0\" size = \"0.04\" rgba = \"0.14 0.16 0.16 1\" material = \"robots\"", Info.geomName, -0.5 * Info.len, 0.5 * Info.len)
        }
        // write sub bodies
        for(int i = 1; i < this->m_nBodyNum; i++) fnvWriteBody(i);
        int nRankDown = this->m_stBodysInfo[this->m_nBodyNum - 1].rank; // write end body
        for(int i = 0; i < nRankDown + 1; i++) { 
            int rank = nRankDown - i;
            __BodyTab(rank) __End("body")
        }
        
        __Tab __End("worldbody") __Etr
    }

    void fnvWriteSensor(int nBodyNum) {
        auto & Info = this->m_stBodysInfo[nBodyNum];
        if(Info.jointType == fs) {
            __Tab __Tab __LineIn2("force name = \"%s\" site = \"%s\" cutoff = \"2000.0\"", Info.jointName[1], Info.jointName[0])
            __Tab __Tab __LineIn2("torque name = \"%s\" site = \"%s\" cutoff = \"2000.0\"", Info.jointName[2], Info.jointName[0])
        }
    }

    void fnvWriteActuator(int nJointNum) {
        __Tab __Tab __LineIn2("position name = \"%s\" gear = \"100.00000\" joint = \"%s\"", this->m_cptJointsList[nJointNum], this->m_cptJointsList[nJointNum])
    }

    void fnvWriteContact(int nExContactNum) {
        __Tab __Tab __LineIn2("exclude body1 = \"%s\" body2 = \"%s\"", this->m_cptExContactList[nExContactNum][0], this->m_cptExContactList[nExContactNum][1])
    }

    void fnvWriteSettings() {
        // write sensors
        __Tab __Start("sensor")
        for(int i = 0; i < this->m_nBodyNum; i++) fnvWriteSensor(i);
        __Tab __End("sensor") __Etr
        // write actuators   
        __Tab __Start("actuator")
        for(int i = 0; i < this->m_nJointNum; i++) fnvWriteActuator(i);
        __Tab __End("actuator") __Etr
        // write contact
        __Tab __Start("contact")
        for(int i = 0; i < this->m_nExContactNum; i++) fnvWriteContact(i);
        __Tab __End("contact") __Etr
        __End("mujoco")
    }

    void fnvCloseFile() {
        fclose(this->m_file);
    }
};