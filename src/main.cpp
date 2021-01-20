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
#include <i2c/smbus.h>



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

int main(int argc, const char **argv)
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
  spdlog::info("Hello, {}!", "World");

  fmt::print("Hello, from {}\n", "{fmt}");



  /*
     Now, you have to decide which adapter you want to access. You should
     inspect /sys/class/i2c-dev/ or run "i2cdetect -l" to decide this.
     Adapter numbers are assigned somewhat dynamically, so you can not
     assume much about them. They can even change from one boot to the next.

     Next thing, open the device file, as follows:
     */
  int file;
  int adapter_nr = 2; /* probably dynamically determined */
  char filename[20];

  snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
  file = open(filename, O_RDWR);
  if (file < 0) {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    exit(1);
  }

  //  When you have opened the device, you must specify with what device
  //    address you want to communicate:

  int addr = 0x40; /* The I2C address */

  if (ioctl(file, I2C_SLAVE, addr) < 0) {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    exit(1);
  }

  //  Well, you are all set up now. You can now use SMBus commands or plain
  //    I2C to communicate with your device. SMBus commands are preferred if
  //   the device supports them. Both are illustrated below.

  __u8 reg = 0x10; /* Device register to access */
  __s32 res;
  char buf[10];

  /* Using SMBus commands */
  res = i2c_smbus_read_word_data(file, reg);
  if (res < 0) {
    /* ERROR HANDLING: i2c transaction failed */
  } else {
    /* res contains the read word */
  }

  /*
   * Using I2C Write, equivalent of
   * i2c_smbus_write_word_data(file, reg, 0x6543)
   */
  buf[0] = reg;
  buf[1] = 0x43;
  buf[2] = 0x65;
  if (write(file, buf, 3) != 3) {
    /* ERROR HANDLING: i2c transaction failed */
  }

  /* Using I2C Read, equivalent of i2c_smbus_read_byte(file) */
  if (read(file, buf, 1) != 1) {
    /* ERROR HANDLING: i2c transaction failed */
  } else {
    /* buf[0] contains the read byte */
  }


}
