typedef struct waypts
{
    double x;
    double y;
    double z;
}waypts;

typedef struct waypts_f
{
    float x;
    float y;
    float z;
}waypts_f;

typedef struct UAVpose
{
    double x;
    double y;
    double z;
    double ow;
    double ox;
    double oy;
    double oz;
}UAVpose;

typedef struct frame_d
{
    double x;
    double y;
    bool got;
}frame_d;

