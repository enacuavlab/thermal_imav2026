#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <CLI/CLI.hpp>
#include <zmq.h>
#include <thermal.pb.h>
extern "C" {
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
}

namespace {

constexpr std::uint8_t kDefaultAddress = 0x33;
constexpr float kDefaultEmissivity = 1.0F;
constexpr float kDefaultReflectedTemperature = 23.0F;
constexpr int kDefaultRefreshRateMode = 2; // 2 Hz subpage rate, 1 full frame/s.
constexpr int kDefaultFrameCount = 0;      // 0 means run forever.
constexpr const char *kDefaultI2cDevice = "/dev/i2c-1";

struct Stats {
    float minimum = std::numeric_limits<float>::quiet_NaN();
    float average = std::numeric_limits<float>::quiet_NaN();
    float maximum = std::numeric_limits<float>::quiet_NaN();
    int count = 0;
};

struct Options {
    std::string device = kDefaultI2cDevice;
    std::string publishEndpoint;
    std::uint8_t address = kDefaultAddress;
    int refreshRateMode = kDefaultRefreshRateMode;
    float emissivity = kDefaultEmissivity;
    float reflectedTemperature = kDefaultReflectedTemperature;
    int frameCount = kDefaultFrameCount;
    bool setChessMode = true;
};

int parseAddress(const std::string &value)
{
    char *end = nullptr;
    long parsed = std::strtol(value.c_str(), &end, 0);
    if (end == value.c_str() || *end != '\0') {
        throw std::invalid_argument("invalid address: " + value);
    }
    if (parsed < 0 || parsed > 0x7F) {
        throw std::invalid_argument("address must be a 7-bit I2C address");
    }
    return static_cast<int>(parsed);
}

Options parseOptions(int argc, char **argv)
{
    Options options;
    std::string address = "0x33";
    bool interleaved = false;
    CLI::App app{"Print MLX90640 min/avg/max temperatures."};

    app.add_option("--device", options.device, "Linux I2C device")
        ->default_val(kDefaultI2cDevice);
    app.add_option("--publish", options.publishEndpoint, "ZeroMQ PUB endpoint, for example tcp://*:5555");
    app.add_option("--address", address, "I2C address")
        ->default_val(address);
    app.add_option("--refresh-mode", options.refreshRateMode, "MLX refresh mode: 0=.5Hz 1=1Hz 2=2Hz 3=4Hz 4=8Hz 5=16Hz 6=32Hz 7=64Hz")
        ->default_val(kDefaultRefreshRateMode)
        ->check(CLI::Range(0, 7));
    app.add_option("--emissivity", options.emissivity, "Emissivity")
        ->default_val(kDefaultEmissivity)
        ->check(CLI::Range(0.000001F, 1.0F));
    app.add_option("--tr", options.reflectedTemperature, "Reflected temperature in Celsius")
        ->default_val(kDefaultReflectedTemperature);
    app.add_option("--frames", options.frameCount, "Number of frames to print, 0 means forever")
        ->default_val(kDefaultFrameCount)
        ->check(CLI::NonNegativeNumber);
    app.add_flag("--interleaved", interleaved, "Use interleaved mode instead of chess mode");

    app.footer("Example:\n  mlx90640_stats --device /dev/i2c-17 --address 0x33");
    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError &error) {
        std::exit(app.exit(error));
    }

    options.setChessMode = !interleaved;
    options.address = static_cast<std::uint8_t>(parseAddress(address));
    return options;
}

void checkMlx(int error, const std::string &operation)
{
    if (error < 0) {
        throw std::runtime_error(operation + " failed with MLX90640 error " + std::to_string(error));
    }
}


void send_ecal(eCAL::protobuf::CPublisher<ThermalData>* pub, const Stats &stats, const float *temperatures, int frame, int subpage) {
  ThermalData msg;
  msg.set_subpage(subpage);
  msg.set_rows(24);
  msg.set_cols(32);
  for (int i = 0; i < MLX90640_PIXEL_NUM; i++) {
      float value = temperatures[i];
      if (std::isfinite(value)) {
          msg.add_data(value);
      } else {
          msg.add_data(0);
      }
  }
  pub->Send(msg);


}

class ZmqPublisher {
public:
    explicit ZmqPublisher(const std::string &endpoint)
    {
        if (endpoint.empty()) {
            return;
        }

        context_ = zmq_ctx_new();
        if (context_ == nullptr) {
            throw std::runtime_error("failed to create ZeroMQ context");
        }

        socket_ = zmq_socket(context_, ZMQ_PUB);
        if (socket_ == nullptr) {
            throw std::runtime_error("failed to create ZeroMQ PUB socket");
        }

        if (zmq_bind(socket_, endpoint.c_str()) != 0) {
            throw std::runtime_error("failed to bind ZeroMQ endpoint: " + endpoint);
        }
    }

    ~ZmqPublisher()
    {
        if (socket_ != nullptr) {
            zmq_close(socket_);
        }
        if (context_ != nullptr) {
            zmq_ctx_term(context_);
        }
    }

    ZmqPublisher(const ZmqPublisher &) = delete;
    ZmqPublisher &operator=(const ZmqPublisher &) = delete;

    void publish(const Stats &stats, const float *temperatures, int frame, int subpage)
    {
        if (socket_ == nullptr) {
            return;
        }

        std::ostringstream payload;
        payload << std::fixed << std::setprecision(3)
                << "{\"frame\":" << frame
                << ",\"subpage\":" << subpage
                << ",\"min\":" << stats.minimum
                << ",\"avg\":" << stats.average
                << ",\"max\":" << stats.maximum
                << ",\"count\":" << stats.count
                << ",\"width\":" << MLX90640_LINE_SIZE
                << ",\"height\":" << MLX90640_LINE_NUM
                << ",\"temperatures\":[";

        for (int i = 0; i < MLX90640_PIXEL_NUM; i++) {
            if (i > 0) {
                payload << ',';
            }
            float value = temperatures[i];
            if (std::isfinite(value)) {
                payload << value;
            } else {
                payload << "null";
            }
        }

        payload << "]}";
        std::string message = payload.str();

        if (zmq_send(socket_, message.data(), message.size(), 0) < 0) {
            throw std::runtime_error("failed to publish ZeroMQ message");
        }
    }

private:
    void *context_ = nullptr;
    void *socket_ = nullptr;
};

Stats calculateStats(const float *temperatures)
{
    Stats stats;
    float minimum = std::numeric_limits<float>::infinity();
    float maximum = -std::numeric_limits<float>::infinity();
    double sum = 0.0;
    int count = 0;

    for (int i = 0; i < MLX90640_PIXEL_NUM; i++) {
        float value = temperatures[i];
        if (!std::isfinite(value)) {
            continue;
        }

        minimum = std::min(minimum, value);
        maximum = std::max(maximum, value);
        sum += value;
        count++;
    }

    if (count == 0) {
        return stats;
    }

    stats.minimum = minimum;
    stats.average = static_cast<float>(sum / count);
    stats.maximum = maximum;
    stats.count = count;
    return stats;
}

void printStats(const Stats &stats)
{
    if (stats.count == 0) {
        std::cout << "no finite temperatures\n";
        return;
    }

    std::cout
        << std::fixed << std::setprecision(2)
        << "min=" << std::setw(7) << stats.minimum << " C  "
        << "avg=" << std::setw(7) << stats.average << " C  "
        << "max=" << std::setw(7) << stats.maximum << " C\n";
}

} // namespace

int main(int argc, char **argv)
{
    try {
        Options options = parseOptions(argc, argv);
        std::uint16_t eeprom[MLX90640_EEPROM_DUMP_NUM] = {};
        std::uint16_t frameData[MLX90640_PIXEL_NUM + MLX90640_AUX_NUM + 2] = {};
        float temperatures[MLX90640_PIXEL_NUM] = {};
        paramsMLX90640 params = {};
        ZmqPublisher publisher(options.publishEndpoint);


	 eCAL::Initialize("MLX90640_driver");
	 eCAL::protobuf::CPublisher<ThermalData> ecal_publisher("mlx90640_data");

        if (setenv("MLX90640_I2C_DEVICE", options.device.c_str(), 1) != 0) {
            throw std::runtime_error("failed to set MLX90640_I2C_DEVICE");
        }
        MLX90640_I2CInit();

        checkMlx(MLX90640_DumpEE(options.address, eeprom), "EEPROM dump");
        checkMlx(MLX90640_ExtractParameters(eeprom, &params), "parameter extraction");

        if (options.setChessMode) {
            checkMlx(MLX90640_SetChessMode(options.address), "set chess mode");
        } else {
            checkMlx(MLX90640_SetInterleavedMode(options.address), "set interleaved mode");
        }

        checkMlx(
            MLX90640_SetRefreshRate(options.address, static_cast<std::uint8_t>(options.refreshRateMode)),
            "set refresh rate"
        );

        for (int frame = 0; options.frameCount == 0 || frame < options.frameCount; frame++) {
            int subpage = MLX90640_GetFrameData(options.address, frameData);
            checkMlx(subpage, "read frame data");

            MLX90640_CalculateTo(
                frameData,
                &params,
                options.emissivity,
                options.reflectedTemperature,
                temperatures
            );

            Stats stats = calculateStats(temperatures);
            printStats(stats);
            publisher.publish(stats, temperatures, frame, subpage);
            send_ecal(&ecal_publisher, stats, temperatures, frame, subpage);
        }

        return 0;
    } catch (const std::exception &error) {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
