#include <Hubo_Tech.h>

int main(int argc, char **argv)
{
    Hubo_Tech hubo;
    hubo.loadURDFModel("/home/rapierevite/hubo-motion-rt/src/dart_Lite/urdf/huboplus.urdf");
    printf("loaded\n");
    return 0;

}
