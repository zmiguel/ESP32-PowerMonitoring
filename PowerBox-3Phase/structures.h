// Structs
//
// These are held in RAM, one malloc'd node per sample, so their size directly
// sets how deep the offline backlog can go before the heap floor starts
// trimming it. Nothing constant is stored per sample: the phase name comes from
// phase_names[] (indexed by position in `data`) and the metric from `metric`,
// both resolved at serialization time.
struct Data {
    float current;
    float voltage;
    float power;
    float energy;
    float frequency;
    float power_factor;
    // False when the sensor could not be read. Invalid entries are dropped
    // during serialization instead of being sent as NaN (which ArduinoJson
    // emits as `null`, and the ingest API rejects as a missing field).
    bool valid;
};

struct Esp {
    int16_t rssi;
    uint32_t acq_time;
};

struct Payload {
    // Unix epoch milliseconds, UTC. 0 means the clock was not synced when the
    // sample was taken - such a payload is sent immediately with backlog=false
    // so the server stamps it on arrival, and is never queued.
    int64_t time_ms;
    Data data[3];
    Esp esp;
};
