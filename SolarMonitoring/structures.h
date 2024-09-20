// Structs
struct Data {
    char phase[32];
    float current;
    float voltage;
    float power;
    float energy;
    float frequency;
    float power_factor;
};

struct Esp {
    int rssi;
    unsigned long acq_time;
};

struct Payload {
    char time[32];
    char metric[32];
    Data data[3];
    Esp esp;
};
