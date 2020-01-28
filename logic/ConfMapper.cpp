#include <ConfMapper.h>

#include <cstdint>
#include <fstream>

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>

namespace ros { namespace pike { namespace logic {

Conf ConfMapper::Load(const std::string& filename)
{
    std::ifstream file{filename};
    rapidjson::IStreamWrapper json_file{file};

    rapidjson::Document doc;
    doc.ParseStream(json_file);

    Conf res;
    
    res.object_length = doc["object_length"].GetDouble();

    const auto& daq = doc["daq"];
    res.daq.slot = daq["slot"].GetInt();
    res.daq.common_gnd = daq["common_gnd"].GetBool();
    res.daq.adc_rate = daq["adc_rate"].GetDouble();

    const auto& depthometer = doc["depthometer"];
    const auto& port_name = depthometer["port_name"];
    res.depthometer.port_name = std::string{port_name.GetString(), port_name.GetStringLength()};
    res.depthometer.baud_rate = depthometer["baud_rate"].GetInt();

    res.ender1.pin = static_cast<uint16_t>(doc["ender1"]["pin"].GetInt());

    res.ender2.pin = static_cast<uint16_t>(doc["ender2"]["pin"].GetInt());

    const auto& inclinometer = doc["inclinometer"];
    res.inclinometer.x_channel = static_cast<uint16_t>(inclinometer["x_channel"].GetInt());
    res.inclinometer.y_channel = static_cast<uint16_t>(inclinometer["y_channel"].GetInt());
    const auto& trans_table_file = inclinometer["trans_table_file"];
    res.inclinometer.trans_table_file = std::string{trans_table_file.GetString(), trans_table_file.GetStringLength()};

    const auto& mover = doc["mover"];
    res.mover.pwm_pin = static_cast<uint16_t>(mover["pwm_pin"].GetInt());
    res.mover.dir_pin = static_cast<uint16_t>(mover["dir_pin"].GetInt());

    const auto& odometer = doc["odometer"];
    res.odometer.a_channel = static_cast<uint16_t>(odometer["a_channel"].GetInt());
    res.odometer.b_channel = static_cast<uint16_t>(odometer["b_channel"].GetInt());
    res.odometer.threshold = odometer["threshold"].GetDouble();
    res.odometer.distance_per_pulse = odometer["distance_per_pulse"].GetDouble();

    const auto& rotator = doc["rotator"];
    res.rotator.en_pin = static_cast<uint16_t>(rotator["en_pin"].GetInt());
    res.rotator.step_pin = static_cast<uint16_t>(rotator["step_pin"].GetInt());
    res.rotator.dir_pin = static_cast<uint16_t>(rotator["dir_pin"].GetInt());
    res.rotator.mx_pin = static_cast<uint16_t>(rotator["mx_pin"].GetInt());
    res.rotator.steps_per_msr = static_cast<uint32_t>(rotator["steps_per_msr"].GetInt());
    res.rotator.steps_per_view = static_cast<uint32_t>(rotator["steps_per_view"].GetInt());

    const auto& remote = doc["remote"];
    res.remote.port = static_cast<uint16_t>(remote["port"].GetInt());

    return res;
}

}}}
