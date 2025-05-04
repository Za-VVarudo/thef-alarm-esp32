#include <ArduinoJson.h>
enum class EventType {

    DOOR_PRESSURE_DETECTED,
    VISITOR_DETECTED,
    NONE

};

class Event {
    public:
        EventType event_type = EventType::NONE;
        int camera_angle;
        String device_name;
        double distance;
        bool light_turned_on;
        double pressure;
};