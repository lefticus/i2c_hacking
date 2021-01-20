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


auto read_byte(int device, std::uint8_t reg) -> std::uint8_t {
  const auto res = i2c_smbus_read_byte_data(device, reg);
  if (res < 0) {
    spdlog::error("Error reading byte of data from {} (err result: {})", reg, res);
  }
  return static_cast<std::uint8_t>(res);
}

auto write_byte(int device, std::uint8_t reg, std::uint8_t value) {
  const auto res = i2c_smbus_write_byte_data(device, reg, value);
  if (res < 0) {
    spdlog::error("Error writing byte of data reg {} (err result: {})", reg, res);
  }
}

auto send_command(int device, std::uint8_t value) {
  const auto res = i2c_smbus_write_byte_data(device, 0, value);
  if (res < 0) {
    spdlog::error("Error writing command data '{}' (err result: {})", value, res);
  }
}


constexpr auto SSD1306_I2C_ADDRESS = 0x3C;    // 011110+SA0+RW - 0x3C or 0x3D
constexpr auto SSD1306_SETCONTRAST = 0x81;
constexpr auto SSD1306_DISPLAYALLON_RESUME = 0xA4;
constexpr auto SSD1306_DISPLAYALLON = 0xA5;
constexpr auto SSD1306_NORMALDISPLAY = 0xA6;
constexpr auto SSD1306_INVERTDISPLAY = 0xA7;
constexpr auto SSD1306_DISPLAYOFF = 0xAE;
constexpr auto SSD1306_DISPLAYON = 0xAF;
constexpr auto SSD1306_SETDISPLAYOFFSET = 0xD3;
constexpr auto SSD1306_SETCOMPINS = 0xDA;
constexpr auto SSD1306_SETVCOMDETECT = 0xDB;
constexpr auto SSD1306_SETDISPLAYCLOCKDIV = 0xD5;
constexpr auto SSD1306_SETPRECHARGE = 0xD9;
constexpr auto SSD1306_SETMULTIPLEX = 0xA8;
constexpr auto SSD1306_SETLOWCOLUMN = 0x00;
constexpr auto SSD1306_SETHIGHCOLUMN = 0x10;
constexpr auto SSD1306_SETSTARTLINE = 0x40;
constexpr auto SSD1306_MEMORYMODE = 0x20;
constexpr auto SSD1306_COLUMNADDR = 0x21;
constexpr auto SSD1306_PAGEADDR = 0x22;
constexpr auto SSD1306_COMSCANINC = 0xC0;
constexpr auto SSD1306_COMSCANDEC = 0xC8;
constexpr auto SSD1306_SEGREMAP = 0xA0;
constexpr auto SSD1306_CHARGEPUMP = 0x8D;
constexpr auto SSD1306_EXTERNALVCC = 0x1;
constexpr auto SSD1306_SWITCHCAPVCC = 0x2;
//  Scrolling constants;
constexpr auto SSD1306_ACTIVATE_SCROLL = 0x2F;
constexpr auto SSD1306_DEACTIVATE_SCROLL = 0x2E;
constexpr auto SSD1306_SET_VERTICAL_SCROLL_AREA = 0xA3;
constexpr auto SSD1306_RIGHT_HORIZONTAL_SCROLL = 0x26;
constexpr auto SSD1306_LEFT_HORIZONTAL_SCROLL = 0x27;
constexpr auto SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL = 0x29;
constexpr auto SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL = 0x2A;

void init_ssd1306(int device)
{
  // 128x32 pixel specific initialization.
  send_command(device, SSD1306_DISPLAYOFF);                    // 0xAE
  send_command(device, SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
  send_command(device, 0x80);                                  // the suggested ratio 0x80
  send_command(device, SSD1306_SETMULTIPLEX);                  // 0xA8
  send_command(device, 0x1F);
  send_command(device, SSD1306_SETDISPLAYOFFSET);              // 0xD3
  send_command(device, 0x0);                                   // no offset
  send_command(device, SSD1306_SETSTARTLINE | 0x0);            // line //0
  send_command(device, SSD1306_CHARGEPUMP);                    // 0x8D
  //if self._vccstate == SSD1306_EXTERNALVCC:
  //    send_command(device, 0x10);
  //else:
  send_command(device, 0x14);
  send_command(device, SSD1306_MEMORYMODE);                    // 0x20
  send_command(device, 0x00);                                  // 0x0 act like ks0108
  send_command(device, SSD1306_SEGREMAP | 0x1);
  send_command(device, SSD1306_COMSCANDEC);
  send_command(device, SSD1306_SETCOMPINS);                    // 0xDA
  send_command(device, 0x02);
  send_command(device, SSD1306_SETCONTRAST);                   // 0x81
  send_command(device, 0x8F);
  send_command(device, SSD1306_SETPRECHARGE);                  // 0xd9
  //if self._vccstate == SSD1306_EXTERNALVCC:
  //    send_command(device, 0x22);
  //else:
  send_command(device, 0xF1);
  send_command(device, SSD1306_SETVCOMDETECT);                 // 0xDB
  send_command(device, 0x40);
  send_command(device, SSD1306_DISPLAYALLON_RESUME);           // 0xA4
  send_command(device, SSD1306_NORMALDISPLAY);                 // 0xA6
  send_command(device, SSD1306_DISPLAYON);                    // 0xAE
  send_command(device, SSD1306_DISPLAYALLON);                    // 0xAE


}


int open_i2c_device(const int deviceid)
{
  int adapter_nr = 1; /* probably dynamically determined */
  char i2cname[20];

  snprintf(i2cname, 19, "/dev/i2c-%d", adapter_nr);
  const auto filehandle = open(i2cname, O_RDWR);
  if (filehandle < 0) {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    spdlog::error("Unable to open i2c device for R/W");
    exit(1);
  }

  if (ioctl(filehandle, I2C_SLAVE, deviceid) < 0) {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    spdlog::error("Unable to set I2C_SLAVE addr to {}", deviceid);
    exit(1);
  }

  return filehandle;
}

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
  constexpr int ds1307_rtc_address = 0x68;

  auto rtc = open_i2c_device(ds1307_rtc_address);

  //  Well, you are all set up now. You can now use SMBus commands or plain
  //    I2C to communicate with your device. SMBus commands are preferred if
  //   the device supports them. Both are illustrated below.

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


  auto oled = open_i2c_device(0x3c);
  init_ssd1306(oled);


}
