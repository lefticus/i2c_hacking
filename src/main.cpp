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
#include <array>

#include <spdlog/spdlog.h>
#include <docopt/docopt.h>

#include <linux/i2c-dev.h>

extern "C" {
#include <i2c/smbus.h>
}

#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <fmt/chrono.h>

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

auto read_block(int device, std::uint8_t reg) -> std::pair<int, std::array<std::uint8_t, 32>> {
  std::pair<int, std::array<std::uint8_t, 32>> result;
  result.first = i2c_smbus_read_block_data(device, reg, result.second.data());
  if (result.first < 0) {
    spdlog::error("Error reading block of data from {} (err result: {})", reg, result.first);
  }
  return result;
}


void write_byte(int device, std::uint8_t reg, std::uint8_t value) {
  const auto res = i2c_smbus_write_byte_data(device, reg, value);
  if (res < 0) {
    spdlog::error("Error writing byte of data reg {} (err result: {})", reg, res);
  }
}

void send_command(int device, std::uint8_t value) {
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


int open_i2c_device(const int adapter_nr, const int deviceid)
{
  const auto filehandle = open(fmt::format("/dev/i2c-{}", adapter_nr).c_str(), O_RDWR);
  if (filehandle < 0) {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    spdlog::error("Unable to open i2c-{} device for R/W", adapter_nr);
    exit(1);
  }

  if (ioctl(filehandle, I2C_SLAVE, deviceid) < 0) {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    spdlog::error("Unable to set I2C_SLAVE addr to {}", deviceid);
    exit(1);
  }

  return filehandle;
}

struct i2c_device
{
  int handle{};
  i2c_device(const int adapter_nr, const int deviceid)
    : handle(open_i2c_device(adapter_nr, deviceid))
  {
  }

  ~i2c_device() {
    ::close(handle);
  }

  i2c_device &operator=(const i2c_device &) = delete;
  i2c_device &operator=(i2c_device &&) = delete;
  i2c_device(const i2c_device &) = delete;
  i2c_device(i2c_device &&) = delete;

  auto read_byte(std::uint8_t reg) const -> std::uint8_t {
    return ::read_byte(handle, reg);
  }

  auto read_block(std::uint8_t reg) const {
    return ::read_block(handle, reg);
  }


  void write_byte(std::uint8_t reg, std::uint8_t value) {
    ::write_byte(handle, reg, value);
  }

  void send_command(std::uint8_t value) {
    ::send_command(handle, value);
  }
};

struct ds1307_rtc : i2c_device
{
  static constexpr int ds1307_rtc_address = 0x68;

  ds1307_rtc(const int adapter) : i2c_device{adapter, ds1307_rtc_address}
  {
  }

  std::array<std::uint8_t, 32> buffer{};

  void sync_buffer() {
    for(std::uint8_t offset = 0; offset < 10; ++offset) {
      buffer[offset] = read_byte(offset);
    }
    //buffer = read_block(0x00).second;
  }

  [[nodiscard]] constexpr std::uint8_t get_buffered_byte(std::size_t offset) const noexcept {
    return buffer[offset];
  }


  [[nodiscard]] constexpr static auto time_field(const std::uint8_t input) noexcept {
      return (0b1111 & input) + ((input >> 4) & 0b1111) * 10;
  };

  [[nodiscard]] int seconds() const {
    return time_field(static_cast<std::uint8_t>(0b0111'1111 & get_buffered_byte(0x00)));
  }

  [[nodiscard]] int minutes() const {
    return time_field(get_buffered_byte(0x01));
  }

  [[nodiscard]] int hours() const {
    return time_field(static_cast<std::uint8_t>(0b111111 & get_buffered_byte(0x02)));
  }
  
  [[nodiscard]] int day_of_week() const {
    return get_buffered_byte(0x03);
  }

  [[nodiscard]] int day_of_month() const {
    return time_field(get_buffered_byte(0x04));
  }

  [[nodiscard]] int month() const {
    return time_field(get_buffered_byte(0x05));
  }

  [[nodiscard]] int year() const {
    return time_field(get_buffered_byte(0x06)) + 2000;
  }



  std::chrono::system_clock::time_point current_time() const 
  {
    std::tm time;
    time.tm_sec = seconds();
    time.tm_min = minutes();
    time.tm_hour = hours();
    time.tm_mday = day_of_month();
    time.tm_mon = month() - 1;
    time.tm_year = year() - 1970;
    time.tm_wday = day_of_week() - 1;

    return std::chrono::system_clock::from_time_t(std::mktime(&time));
  }


  template<std::size_t Offset=0, typename DataType>
    void write_object(const DataType &data)
    {
      static_assert(sizeof(DataType)+Offset <= 56);
      static_assert(std::is_trivial_v<DataType>);

      const auto *ptr = reinterpret_cast<const std::uint8_t *>(&data);

      constexpr auto start = 0x08;

      for (std::size_t i = 0; i < sizeof(DataType); ++i) {
        write_byte(static_cast<std::uint8_t>(start + Offset + i), *std::next(ptr, static_cast<std::ptrdiff_t>(i)));
      }
    }


  template<std::size_t Offset=0, typename DataType>
    DataType read_object()
    {
      static_assert(sizeof(DataType)+Offset <= 56);
      static_assert(std::is_trivial_v<DataType>);

      DataType data;
      auto *ptr = reinterpret_cast<std::uint8_t *>(&data);

      constexpr auto start = 0x08;

      for (std::size_t i = 0; i < sizeof(DataType); ++i) {
        *std::next(ptr, static_cast<std::ptrdiff_t>(i)) = read_byte(static_cast<std::uint8_t>(start + i + Offset));
      }

      return data;
    }
};


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

  ds1307_rtc real_time_clock(1);

  const std::array<char, 5> data{'H', 'e', 'l', 'l', 'o'};
  real_time_clock.write_object<0>(data);

  fmt::print("Pre sync:  Current Time: {:02}:{:02}:{:02}\n", real_time_clock.hours(), real_time_clock.minutes(), real_time_clock.seconds());
  real_time_clock.sync_buffer();
  fmt::print("Post sync: Current Time: {:02}:{:02}:{:02}\n", real_time_clock.hours(), real_time_clock.minutes(), real_time_clock.seconds());

//  fmt::print("{}", real_time_clock.current_time());

  
  const auto read_data = real_time_clock.read_object<0, std::array<char, 5>>();
  
  for (const auto c : read_data) {
    fmt::print("Read: '{}'\n", c);
  }

  auto oled = open_i2c_device(1, 0x3c);
  init_ssd1306(oled);


}
