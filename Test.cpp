#include <DModelMaker.hpp>

c_MMaker cMMaker7p2("bhr7p2_auto", 0.004, 1);

void main() {
    double dOriPos[3] = {0.0, 0.0, 0.84};
    double dIner[3] = {0.0000001, 0.0000001, 0.0000001};
    cMMaker7p2.fnvAddBase("midbody", dOriPos, 0.0000001, dIner, 0.0);
    cMMaker7p2.fnvWriteXML();
}