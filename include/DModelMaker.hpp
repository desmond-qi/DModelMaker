// DModelMaker.hpp
// 20230314 dcc <3120195094@bit.edu.cn>
#include <stdio.h>
#include <iostream>
#include <string.h>

#define __MaxStrLen 60
#define __MaxJointNum 50
#define __Tab                   fprintf(this->m_file, "\t");
#define __Etr                   fprintf(this->m_file, "\n");
#define __Start(content)        fprintf(this->m_file, "<"); fprintf(this->m_file, content); fprintf(this->m_file, ">"); __Etr
#define __End(content)          fprintf(this->m_file, "<"); fprintf(this->m_file, content); fprintf(this->m_file, "/>"); __Etr __Etr
#define __Line(content)         fprintf(this->m_file, "<"); fprintf(this->m_file, content); fprintf(this->m_file, "/>"); __Etr
#define __LineIn1(content, input) \
        fprintf(this->m_file, "<"); fprintf(this->m_file, content, input); fprintf(this->m_file, "/>"); __Etr
#define __LineIn4(content, input1, input2, input3, input4) \
        fprintf(this->m_file, "<"); fprintf(this->m_file, content, input1, input2, input3, input4); fprintf(this->m_file, "/>"); __Etr
#define __LineIn7(content, input1, input2, input3, input4, input5, input6, input7) \
        fprintf(this->m_file, "<"); fprintf(this->m_file, content, input1, input2, input3, input4, input5, input6, input7); fprintf(this->m_file, "/>"); __Etr

enum en_JointType {
    pin, gim, bal
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
    }
    ~c_MMaker() {
        this->fnvCloseFile();
    }
    void fnvAddBase(
        char * cptBodyName, 
        double dOriPos[3],
        double dMass, 
        double dIner[3], 
        double dLinkLength) {
        auto & Info = this->m_stJointsInfo[this->m_nBodyNum];
        strcpy(Info.jointName[0], "root"); // copy the joint name
        strcpy(Info.bodyName, cptBodyName); // copy the body name
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
        char * cptBodyName, 
        char * cptInbName,
        double dMass, 
        double dIner[3], 
        double dLinkLength,
        char * cptJointName, 
        double dAxis[3], 
        double dInb2Joint[3], 
        double dBody2Joint[3]) {
        this->m_nBodyNum++; // add the body number
        this->m_nJointNum++; // add the joint number
        auto & Info = this->m_stJointsInfo[this->m_nBodyNum];
        strcpy(Info.jointName[0], cptJointName); // copy the joint name
        strcpy(Info.bodyName, cptBodyName); // copy the body name
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
        for(int i = 0; i < __MaxJointNum; i++) {
            if(strcmp(cptInbName, this->m_stJointsInfo[i].bodyName) == 0) {
                Info.rank += this->m_stJointsInfo[i].rank; 
                break;
            }// find inbord and get rank
            if(i >= this->m_nBodyNum) { // didn't find the same inbord body name
                _STD cout << "Wrong inbord body name" << Info.bodyName << _STD endl;
                break;
            }
        }
        Info.BodytNo = this->m_nBodyNum; // body number
        Info.jointType = pin; // joint type
    }
    void fnvAddGimbal(
        char * cptBodyName, 
        char * cptInbName,
        double dMass, 
        double dIner[3], 
        double dLinkLength,
        char * cptJointName1, 
        char * cptJointName2, 
        double dAxis1[3], 
        double dAxis2[3], 
        double dInb2Joint[3], 
        double dBody2Joint[3]) {
        this->m_nBodyNum++; // add the body number
        this->m_nJointNum += 2; // add the joint number
        auto & Info = this->m_stJointsInfo[this->m_nBodyNum];
        strcpy(Info.jointName[0], cptJointName1); // copy the joint name
        strcpy(Info.jointName[2], cptJointName2); // copy the joint name
        strcpy(Info.bodyName, cptBodyName); // copy the body name
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
        for(int i = 0; i < __MaxJointNum; i++) {
            if(strcmp(cptInbName, this->m_stJointsInfo[i].bodyName) == 0) {
                Info.rank += this->m_stJointsInfo[i].rank; 
                break;
            }// find inbord and get rank
            if(i >= this->m_nBodyNum) { // didn't find the same inbord body name
                _STD cout << "Wrong inbord body name" << Info.bodyName << _STD endl;
                break;
            }
        }
        Info.BodytNo = this->m_nBodyNum; // body number
        Info.jointType = gim; // joint type
    }
    void fnvAddBall(
        char * cptBodyName, 
        char * cptInbName,
        double dMass, 
        double dIner[3], 
        double dLinkLength,
        char * cptJointName1, 
        char * cptJointName2, 
        char * cptJointName3, 
        double dAxis1[3], 
        double dAxis2[3], 
        double dAxis3[3], 
        double dInb2Joint[3], 
        double dBody2Joint[3]) {
        this->m_nBodyNum++; // add the body number
        this->m_nJointNum += 3; // add the joint number
        auto & Info = this->m_stJointsInfo[this->m_nBodyNum];
        strcpy(Info.jointName[0], cptJointName1); // copy the joint name
        strcpy(Info.jointName[2], cptJointName2); // copy the joint name
        strcpy(Info.jointName[2], cptJointName3); // copy the joint name
        strcpy(Info.bodyName, cptBodyName); // copy the body name
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
        for(int i = 0; i < __MaxJointNum; i++) {
            if(strcmp(cptInbName, this->m_stJointsInfo[i].bodyName) == 0) {
                Info.rank += this->m_stJointsInfo[i].rank; 
                break;
            }// find inbord and get rank
            if(i >= this->m_nBodyNum) { // didn't find the same inbord body name
                _STD cout << "Wrong inbord body name" << Info.bodyName << _STD endl;
                break;
            }
        }
        Info.BodytNo = this->m_nBodyNum; // body number
        Info.jointType = bal; // joint type
    }
    void fnvWriteXML() {
        this->fnvWriteAsset();
        this->fnvWriteWorld();
    }
private:
    int m_nJointNum;
    int m_nBodyNum;
    char m_cptModelName[__MaxStrLen];
    double m_dTimeStep;
    int m_nGravityFlag;
    FILE * m_file;
    st_JointBody m_stJointsInfo[__MaxJointNum];
    void fnvOpenFile() {
        char wchFileName[__MaxStrLen];
        strcpy_s(wchFileName, m_cptModelName);
        strcat_s(wchFileName, ".txt");
        this->m_file = fopen(wchFileName, "w"); 
    }
    void fnvWriteDefault() {
        fprintf(this->m_file, "<mujoco model = \"%s\">", this->m_cptModelName); __Etr __Etr

        __Tab __Line("compiler inertiafromgeom = \"false\" angle = \"radian\""); __Etr __Etr

        __Tab __Start("default") 
        __Tab __Tab __Line("joint limited = \"true\" stiffness = \"1500\" damping = \"10\" armature = \"2.5\"");
        __Tab __Tab __Line("geom condim = \"4\" material = \"matgeom\"");
        __Tab __Tab __Line("motor ctrlrange = \"-314.0 314.0\" ctrllimited = \"true\"");
        __Tab __Tab __Line("position kp = \"15\"");
        __Tab __End("default") 

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
        __Tab __End("visual")
    }
    void fnvWriteAsset() {
        __Tab __Start("asset")
        __Tab __Tab __Line("texture type=\"skybox\" builtin=\"gradient\" rgb1=\"0.5 0.8 1.0\" rgb2=\"0.4 0.6 0.7\" width=\"512\" height=\"512\"")
        __Tab __Tab __Line("texture name=\"texplane\" type=\"2d\" builtin=\"checker\" rgb1=\".2 .3 .4\" rgb2=\".1 0.15 0.2\" width=\"512\" height=\"512\" mark=\"cross\" markrgb=\".8 .8 .8\"")
        __Tab __Tab __Line("texture name=\"texgeom\" type=\"cube\" builtin=\"flat\" mark=\"cross\" width=\"127\" height=\"1278\" rgb1=\"0.8 0.6 0.4\" rgb2=\"0.8 0.6 0.4\" markrgb=\"1 1 1\" random=\"0.01\"")
        __Tab __Tab __Line("material name=\"matplane\" reflectance=\"0.3\" texture=\"texplane\" texrepeat=\"1 1\" rgba =\"0.9 0.9 0.9 1\"")
        __Tab __Tab __Line("material name=\"matgeom\" texture=\"texgeom\" texuniform=\"true\" rgba=\"0.8 0.6 .4 1\"")
        __Tab __Tab __Line("material name=\"robots\" shininess=\"0.5\" reflectance=\"0.5\"")
        __Tab __End("asset")
    }
    void fnvWriteWorld() {
        __Tab __Start("worldbody")
        __Tab __Tab __Line("geom name = \"floor\" pos = \"0 0 0\" size = \"0 0 .25\" type = \"plane\" material = \"matplane\" condim = \"3\" friction = \"1.3 0.005 0.0001\"")
        __Tab __Tab __Line("light directional = \"true\" diffuse = \".5 .5 .5\" specular = \"0.3 0.3 0.3\" pos = \"0 0 5\" dir = \"-0.25 0 -1\" castshadow = \"false\"")
        __Tab __Tab __Line("light mode = \"targetbodycom\" target = \"midbody\" directional = \"true\" diffuse = \".7 .7 .7\" specular = \"0.3 0.3 0.3\" pos = \"0 0 4.0\" dir = \"0 0 -1\" castshadow = \"false\"") __Etr

        auto & Info = this->m_stJointsInfo[0];
        __Tab __Tab __LineIn4("body name = \"%s\" pos = \"%lf %lf %lf\"", Info.bodyName, Info.i2b[0], Info.i2b[1], Info.i2b[2])
        
        __Tab __End("worldbody")
    }
    void fnvCloseFile() {
        fclose(this->m_file);
    }
};