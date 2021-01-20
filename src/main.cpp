// Docs and notes
//
// RTC: https://datasheets.maximintegrated.com/en/ds/DS1307.pdf
// OLED Driver: https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf
// 9-axis Motion Sensor: https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
// Motion Sensor Register Map: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-9250-Register-Map.pdf
// EEPROM: https://www.st.com/resource/en/datasheet/m24c32-x.pdf
//
//
// OLED Python Examples: https://github.com/adafruit/Adafruit_Python_SSD1306/blob/master/Adafruit_SSD1306/SSD1306.py
//
// RaspberryPi I2C Info: https://raspberry-projects.com/pi/programming-in-c/i2c/using-the-i2c-interface
//
// Kernel.org I2C Documentation: https://www.kernel.org/doc/Documentation/i2c/dev-interface
//

#include <functional>
#include <iostream>

#include <spdlog/spdlog.h>
#include <docopt/docopt.h>

#include <linux/i2c-dev.h>

extern "C" {
#include <i2c/smbus.h>
}

#include <sys/ioctl.h>
#include <fcntl.h>


static constexpr auto USAGE =
R"(Naval Fate.

    Usage:
          naval_fate ship new <name>...
          naval_fate ship <name> move <x> <y> [--speed=<kn>]
          naval_fate ship shoot <x> <y>
          naval_fate mine (set|remove) <x> <y> [--moored | --drifting]
          naval_fate (-h | --help)
          naval_fate --version
 Options:
          -h --help     Show this screen.
          --version     Show version.
          --speed=<kn>  Speed in knots [default: 10].
          --moored      Moored (anchored) mine.
          --drifting    Drifting mine.
)";



int main(/*int argc, const char **argv*/)
{
  /*
     std::map<std::string, docopt::value> args = docopt::docopt(USAGE,
     { std::next(argv), std::next(argv, argc) },
     true,// show help if requested
     "Naval Fate 2.0");// version string

     for (auto const &arg : args) {
     std::cout << arg.first << arg.second << std::endl;
     }
     */

  //Use the default logger (stdout, multi-threaded, colored)
  spdlog::info("Starting i2c Experiments");


  /*
     Now, you have to decide which adapter you want to access. You should
     inspect /sys/class/i2c-dev/ or run "i2cdetect -l" to decide this.
     Adapter numbers are assigned somewhat dynamically, so you can not
     assume much about them. They can even change from one boot to the next.

     Next thing, open the device rtc, as follows:
     */
  int rtc;
  int adapter_nr = 1; /* probably dynamically determined */
  char rtcname[20];

  snprintf(rtcname, 19, "/dev/i2c-%d", adapter_nr);
  rtc = open(rtcname, O_RDWR);
  if (rtc < 0) {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    spdlog::error("Unable to open i2c device for R/W");
    exit(1);
  }

  constexpr int ds1307_rtc_address = 0x68;

  if (ioctl(rtc, I2C_SLAVE, ds1307_rtc_address) < 0) {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    spdlog::error("Unable to set I2C_SLAVE addr to {}", ds1307_rtc_address);
    exit(1);
  }

  //  Well, you are all set up now. You can now use SMBus commands or plain
  //    I2C to communicate with your device. SMBus commands are preferred if
  //   the device supports them. Both are illustrated below.

  auto read_byte = [](int device, std::uint8_t reg) -> std::uint8_t {
    const auto res = i2c_smbus_read_byte_data(device, reg);
    if (res < 0) {
      spdlog::error("Error reading byte of data from {} (err result: {})", reg, res);
    }
    return static_cast<std::uint8_t>(res);
  };

  auto write_byte = [](int device, std::uint8_t reg, std::uint8_t value) {
    const auto res = i2c_smbus_write_byte_data(device, reg, value);
    if (res < 0) {
      spdlog::error("Error writing byte of data reg {} (err result: {})", reg, res);
    }
  };
 
  auto time_field = [](const std::uint8_t input) {
    return (0b1111 & input) + ((input >> 4) & 0b111) * 10;
  };

  // force 24 mode, putting in the same hour as was just read.
  //
  // let's really hope that we don't do this during a race for hours rolling over
  //
  // write_byte(rtc, 0x02, static_cast<std::uint8_t>(0b0011'1111 & read_byte(rtc, 0x02)));
  const auto seconds = read_byte(rtc, 0x00);
  const auto minutes = read_byte(rtc, 0x01);
  const auto hours = 0b111111 & read_byte(rtc, 0x02);

  fmt::print("{:02}:{:02}:{:02}\n", time_field(hours), time_field(minutes), time_field(seconds));

  write_byte(rtc, 0x10, 42);





}
