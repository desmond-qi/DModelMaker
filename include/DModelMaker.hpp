// DModelMaker.hpp
// 20230314 dcc <3120195094@bit.edu.cn>
#include <DBase.hpp>
#include <stdio.h>
#include <iostream>
#include <string>
#include <windows.h>

_D_USING_BASE

#define __MaxStrLen 60
#define __MaxBodyNum 50
#define __MaxJointNum 50
#define __MaxIMUNum 10
#define __MaxFSNum 10
#define __MaxBlockNum 20
#define __Tab                   fprintf(this->m_file, "\t");
#define __BodyTab(rank)         { __Tab __Tab for(int i = 0; i < rank; i++) __Tab }
#define __Etr                   fprintf(this->m_file, "\n");
#define __Start(content)        fprintf(this->m_file, "<"); fprintf(this->m_file, content); fprintf(this->m_file, ">"); __Etr
#define __StartMujoco(content, input) \
            fprintf(this->m_file, "<"); fprintf(this->m_file, content, input); fprintf(this->m_file, ">"); __Etr
#define __StartBody(content, input1, input2, input3, input4) \
            fprintf(this->m_file, "<"); fprintf(this->m_file, content, input1, input2, input3, input4); fprintf(this->m_file, ">"); __Etr
#define __StartBlock(content, input1, input2, input3, input4, input5, input6, input7) \
            fprintf(this->m_file, "<"); fprintf(this->m_file, content, input1, input2, input3, input4, input5, input6, input7); fprintf(this->m_file, ">"); __Etr
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
#define __LineIn5(content, input1, input2, input3, input4, input5) \
            fprintf(this->m_file, "<"); fprintf(this->m_file, content, input1, input2, input3, input4, input5); fprintf(this->m_file, "/>"); __Etr
#define __LineIn7(content, input1, input2, input3, input4, input5, input6, input7) \
            fprintf(this->m_file, "<"); fprintf(this->m_file, content, input1, input2, input3, input4, input5, input6, input7); fprintf(this->m_file, "/>"); __Etr

enum en_JointType {
    pin = 1, gim, bal, fs
};

struct st_Friction {
    double impratio; // default global friction
    double friction[3]; // default feet friction
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

struct st_Block {
    double Mass; // if is zero, making the block static
    double Iner[3]; // rotational inertia
    double geom[3]; // block size
    double pos[3]; // position in the global world
    double rot[3]; // rotation in the global world
    char blockName[__MaxStrLen]; // 
    int blockNo; // the number of the block
};

class c_MMaker {
public:
    int m_nJointNum;
    int m_nBodyNum;
    int m_nIMUNum;
    int m_nFSNum;
    int m_nMotorMod; // 0: position; 1:torque
    st_JointBody m_stBodysInfo[__MaxBodyNum];

    c_MMaker(const char * cptModelName, double dTimeStep, int nMotorMod, int nIfGravity) {
        strcpy_s(this->m_cptModelName, cptModelName);
        this->m_dTimeStep = dTimeStep;
        this->m_nGravityFlag = nIfGravity;
        this->m_nMotorMod = nMotorMod;
        this->m_nBodyNum = 0;
        this->m_nJointNum = 0;
        this->m_nIMUNum = 0;
        this->m_nFSNum = 0;
        this->m_nExContactNum = 0;
        this->m_nBlockNum = 0;
        this->m_nPlaneNum = 0;        
        this->fnvSetFriction(10.0, { 3.8, 0.5, 0.0001 }); // set the default values of the friction setting
        this->fnvOpenFile();
    }
    
    ~c_MMaker() {
        
    }

    void fnvSetFriction(double dImpratio, double3 d3Friction) {
        this->m_stFriction.impratio = dImpratio;
        for(int i = 0; i < 3; i++) this->m_stFriction.friction[i] = d3Friction[i];
    }

    void fnvAddBase(
        char *      cptBodyName, 
        double3     dOriPos,
        double      dMass, 
        double3     dIner, 
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
        double3     dIner, 
        int         nIfGeom,
        double      dLinkLength,
        char *      cptJointName, 
        double3     dAxis,  
        double3     dBody2Joint,
        double3     dInb2Joint) {
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
        double3     dIner, 
        int         nIfGeom,
        double      dLinkLength,
        char *      cptJointName1, 
        char *      cptJointName2, 
        double3     dAxis1, 
        double3     dAxis2, 
        double3     dBody2Joint,
        double3     dInb2Joint) {
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
        double3     dIner, 
        int         nIfGeom,
        double      dLinkLength,
        char *      cptJointName1, 
        char *      cptJointName2, 
        char *      cptJointName3, 
        double3     dAxis1, 
        double3     dAxis2, 
        double3     dAxis3, 
        double3     dBody2Joint,
        double3     dInb2Joint) {
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
        double3     dIner, 
        int         nIfGeom,
        double      dForceSensorHeight,
        char *      cptForceSensorName,
        char *      cptForceName,
        char *      cptTorqueName,
        double3     dSize, 
        double3     dInb2Body) {
        this->m_nBodyNum++; // add the body number
        this->m_nFSNum++; // add the force sensor number
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
        double3     dInb2IMU) {
        this->m_nIMUNum++; // add the IMU number
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

    void fnvBuildBlock(
        double      dMass, 
        double3     d3Geom, 
        double3     d3Pos, 
        double3     d3Euler) {
        auto & Info = this->m_stBlocksInfo[this->m_nBlockNum];
        Info.blockNo = this->m_nBlockNum;
        this->m_nBlockNum++;
        strcpy(Info.blockName, "block_");
        strcat(Info.blockName, ("0" + Info.blockNo));
        Info.Mass = dMass;
        for(int i = 0; i < 3; i++) Info.geom[i] = d3Geom[i], Info.pos[i] = d3Pos[i], Info.rot[i] = d3Euler[i];
        Info.Iner[0] = dMass * (d3Geom[1] * d3Geom[1] + d3Geom[2] * d3Geom[2]) / 12.0;
        Info.Iner[1] = dMass * (d3Geom[2] * d3Geom[2] + d3Geom[0] * d3Geom[0]) / 12.0;
        Info.Iner[2] = dMass * (d3Geom[0] * d3Geom[0] + d3Geom[1] * d3Geom[1]) / 12.0;
    }

    void fnvExContact(char * cptBody1, char * cptBody2) {
        strcpy_s(this->m_cptExContactList[this->m_nExContactNum][0], cptBody1);
        strcpy_s(this->m_cptExContactList[this->m_nExContactNum][1], cptBody2);
        this->m_nExContactNum++;
    }

    void fnvWriteXML() {
        this->m_nBodyNum++; // add the counting number of floating base body
        this->fnvWriteHeader();
        if(this->m_nMotorMod == 0) this->fnvWriteDefault();
        this->fnvWriteOptions();
        this->fnvWriteAsset();
        this->fnvWriteWorld();
        this->fnvWriteSettings();
        Sleep(200);
        this->fnvCloseFile();
        _STD cout << "Model file: [ " << this->m_cptModelName << ".xml ] is generated in /Build/Release!" << _STD endl;
    }

    void fnvDisp() {
        _STD cout << "Number of bodys is: " << this->m_nBodyNum << _STD endl;
        _STD cout << "Number of joints is: " << this->m_nJointNum << _STD endl;
    }

private:
    char m_cptModelName[__MaxStrLen];
    double m_dTimeStep;
    st_Friction m_stFriction;
    int m_nGravityFlag;
    FILE * m_file;
    st_Block m_stBlocksInfo[__MaxBlockNum];
    char m_cptJointsList[__MaxJointNum][__MaxStrLen];
    char m_cptExContactList[100][2][__MaxStrLen];
    int m_nExContactNum;
    int m_nBlockNum;
    int m_nPlaneNum;
    void fnvOpenFile() {
        char wchFileName[__MaxStrLen];
        strcpy_s(wchFileName, m_cptModelName);
        strcat_s(wchFileName, ".xml");
        this->m_file = fopen(wchFileName, "w"); 
    }

    void fnvWriteHeader() {
        __StartMujoco("mujoco model = \"%s\"", this->m_cptModelName) __Etr 

        __Tab __Line("compiler inertiafromgeom = \"false\" angle = \"radian\""); __Etr 
    }

    void fnvWriteDefault() {
        __Tab __Start("default") 
        __Tab __Tab __Line("joint limited = \"true\" stiffness = \"1500\" damping = \"10\" armature = \"2.5\"");
        __Tab __Tab __Line("geom condim = \"4\" material = \"matgeom\"");
        __Tab __Tab __Line("motor ctrlrange = \"-314.0 314.0\" ctrllimited = \"true\"");
        __Tab __Tab __Line("position kp = \"15\"");
        __Tab __End("default") __Etr
    }

    void fnvWriteOptions() {
        __Tab fprintf(this->m_file, "<option timestep = \"%lf\" ", this->m_dTimeStep); fprintf(this->m_file, "apirate = \"144\" iterations = \"50\"/>"); __Etr
        __Tab __LineIn1("option impratio = \"%lf\"", this->m_stFriction.impratio) 
        __Tab __Line("option tolerance = \"1e-10\" solver = \"Newton\"")
        __Tab 
        if(this->m_nGravityFlag) {
            __Line("option jacobian = \"dense\" cone = \"pyramidal\" gravity = \"0.0 0.0 -9.8\"") __Etr //QHX
        }
        else {
            __Line("option jacobian = \"dense\" cone = \"pyramidal\" gravity = \"0.0 0.0 0.0\"") __Etr
        }
        
        __Tab __Line("size nconmax = \"50\" njmax = \"200\" nstack = \"10000\"") __Etr

        __Tab __Start("visual")
        __Tab __Tab __Line("map force = \"0.1\" zfar = \"30\"")
        __Tab __Tab __Line("rgba haze = \"0.1 0.1 0.1 0\"")
        __Tab __Tab __Line("quality shadowsize = \"4096\"")
        __Tab __Tab __Line("global offwidth = \"1600\" offheight = \"1600\"")
        __Tab __Tab __Line("scale forcewidth = \"0.05\" contactwidth = \"0.1\" com = \"0.2\"")
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
                __Tab __BodyTab(Info.rank) __LineIn7("geom name = \"%s\" type = \"box\" size = \"%lf %lf %lf\" rgba = \"0.14 0.16 0.16 1\" material = \"robots\" friction = \"%lf %lf %lf\"", Info.bodyName, Info.b2j[0], Info.b2j[1], Info.b2j[2], this->m_stFriction.friction[0], this->m_stFriction.friction[1], this->m_stFriction.friction[2]) // write geom
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

    void fnvWriteBlocks(int nBlockNum) {
        auto & Info = this->m_stBlocksInfo[nBlockNum];
        __BodyTab(0) __StartBlock("body name = \"%s\" pos = \"%lf %lf %lf\" euler = \"%lf %lf %lf\"", Info.blockName, Info.pos[0], Info.pos[1], Info.pos[2], Info.rot[0], Info.rot[1], Info.rot[2])
        if(Info.Mass > __Mic6) { // respondable
            __Tab __BodyTab(0) __LineIn4("inertial pos = \"0 0 0\" mass = \"%lf\" diaginertia = \"%lf %lf %lf\"", Info.Mass, Info.Iner[0], Info.Iner[1], Info.Iner[2])
            __Tab __BodyTab(0) __LineIn2("freejoint name = \"%s_%s\"", Info.blockName, "freejoint")
            __Tab __BodyTab(0) __LineIn5("geom name = \"%s_%s\" type = \"box\" size = \"%lf %lf %lf\" rgba = \"0.5 0.2 0.2 1\"", Info.blockName, "geom", 0.5 * Info.geom[0], 0.5 * Info.geom[1], 0.5 * Info.geom[2])
        }
        else { // static
            __Tab __BodyTab(0) __Line("inertial pos = \"0 0 0\" mass = \"100\" diaginertia = \"1 1 1\"")
            __Tab __BodyTab(0) __LineIn5("geom name = \"%s_%s\" type = \"box\" size = \"%lf %lf %lf\" rgba = \"0.5 0.2 0.2 1\"", Info.blockName, "geom", 0.5 * Info.geom[0], 0.5 * Info.geom[1], 0.5 * Info.geom[2])
        }
        __BodyTab(0) __End("body")
    }

    void fnvWriteWorld() {
        // write default world body
        __Tab __Start("worldbody")
        __Tab __Tab __Line("geom name = \"floor\" pos = \"0 0 0\" size = \"0 0 .25\" type = \"plane\" material = \"matplane\" condim = \"3\" friction = \"1.3 0.005 0.0001\"")
        __Tab __Tab __Line("light directional = \"true\" diffuse = \".5 .5 .5\" specular = \"0.3 0.3 0.3\" pos = \"0 0 5\" dir = \"-0.25 0 -1\" castshadow = \"false\"")
        __Tab __Tab __Line("light mode = \"targetbodycom\" target = \"uppbody\" directional = \"true\" diffuse = \".7 .7 .7\" specular = \"0.3 0.3 0.3\" pos = \"0 0 4.0\" dir = \"0 0 -1\" castshadow = \"false\"") __Etr
        { // write base body
        auto & Info = this->m_stBodysInfo[0]; 
        __BodyTab(0) __StartBody("body name = \"%s\" pos = \"%lf %lf %lf\"", Info.bodyName, Info.i2b[0], Info.i2b[1], Info.i2b[2])
        __Tab __BodyTab(0) __LineIn1("freejoint name = \"%s\"", Info.jointName)
        __Tab __BodyTab(0) __LineIn4("inertial pos = \"0 0 0\" mass = \"%lf\" diaginertia = \"%lf %lf %lf\"", Info.Mass, Info.Iner[0], Info.Iner[1], Info.Iner[2])
        __Tab __BodyTab(0) __LineIn3("geom name = \"%s\" type = \"capsule\" fromto = \"0 %lf 0 0 %lf 0\" size = \"0.04\" rgba = \"0.14 0.16 0.16 1\" material = \"robots\"", Info.geomName, -0.5 * Info.len, 0.5 * Info.len)
        __Tab __BodyTab(0) __LineIn1("site name = \"%s\" size = \"0.01\" pos = \"0 0 0\"", "imu") //QHX
        }
        // write sub bodies
        for(int i = 1; i < this->m_nBodyNum; i++) fnvWriteBody(i);
        int nRankDown = this->m_stBodysInfo[this->m_nBodyNum - 1].rank; // write end body
        for(int i = 0; i < nRankDown + 1; i++) { 
            int rank = nRankDown - i;
            __BodyTab(rank) __End("body")
        }
        // write blocks
        for(int i = 0; i < this->m_nBlockNum; i++) fnvWriteBlocks(i);
        // write planes

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
        if(this->m_nMotorMod == 0) {
            __Tab __Tab  __LineIn2("position name = \"%s\" gear = \"100.00000\" joint = \"%s\"", this->m_cptJointsList[nJointNum], this->m_cptJointsList[nJointNum])
        }
        else if(this->m_nMotorMod == 1) {
            __Tab __Tab  __LineIn2("motor name = \"%s\" joint = \"%s\"", this->m_cptJointsList[nJointNum], this->m_cptJointsList[nJointNum])
        }
        else _STD cout << "Wrong motor mod!!" << _STD endl;
    }

    void fnvWriteContact(int nExContactNum) {
        __Tab __Tab __LineIn2("exclude body1 = \"%s\" body2 = \"%s\"", this->m_cptExContactList[nExContactNum][0], this->m_cptExContactList[nExContactNum][1])
    }

    void fnvWriteSettings() {
        // write sensors
        __Tab __Start("sensor")
        for(int i = 0; i < this->m_nBodyNum; i++) fnvWriteSensor(i);
        __Tab __Tab __LineIn2("gyro name = \"%s\" site = \"%s\"", "root_gyro", "imu") //QHX
        __Tab __Tab __LineIn2("accelerometer name = \"%s\" site = \"%s\"", "root_accel", "imu")
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