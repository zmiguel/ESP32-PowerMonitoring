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

// Why a read cycle produced nothing usable. The PZEM library reads all ten
// registers in a single Modbus transaction, so a failure is never one bad
// field - it is the whole meter missing for that second.
enum SensorFail : uint8_t {
    SENSOR_OK = 0,
    SENSOR_NO_REPLY,    // no frame came back, or it failed CRC / had the wrong length
    SENSOR_BAD_VALUES   // a frame decoded, but a field is outside a plausible range
};

// One attempt at reading a meter, kept whole so a failure can be reported with
// the values that were actually on the wire rather than just "it failed".
struct SensorReading {
    float current;
    float voltage;
    float power;
    float energy;
    float frequency;
    float pf;
    uint8_t reason;      // SensorFail
    uint8_t badMask;     // bit per field that failed its range check, see field_names[]
    uint16_t durationMs;
};

// Per-meter read health. Deliberately not part of Payload: that is malloc'd
// once per backlogged sample, and this is the same three counters regardless.
// Written only by the sampling loop, read by /status on the async server task,
// hence volatile.
struct SensorHealth {
    volatile uint32_t reads;        // read cycles attempted
    volatile uint32_t failures;     // cycles that produced no valid reading
    volatile uint32_t consecutive;  // consecutive failed cycles, 0 while healthy
    volatile uint32_t lastGoodMs;   // millis() of the last valid reading, 0 = never
    volatile uint32_t lastTxnMs;    // millis() of the last Modbus transaction issued
    volatile uint8_t lastReason;    // SensorFail code of the most recent failure
};
