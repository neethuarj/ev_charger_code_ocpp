#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <pigpio.h>
#include <unistd.h>
#include <csignal>
#include <ctime>
#include <thread>
#include <iomanip>
#include <modbus.h>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <vector>
#include <chrono>

#include "websockets/IWebsocketClient.h"
#include "websockets/WebsocketFactory.h"
#include "rpc/RpcClient.h"
#include <json/json.h>  // If using jsoncpp library
#include "websockets/IWebsocketClient.h"

#include "websockets/Credentials.h"

using namespace std::chrono_literals;
// Ensure you have created the websocket client properly
std::unique_ptr<ocpp::websockets::IWebsocketClient> wsClient;  // Create or initialize your WebSocket client
std::string protocol = "ocpp1.6j";


std::string serverUrl = "ws://46.23.63.60:8080/steve/websocket/CentralSystemService/";
std::string stationID = "EVSE_01";

class ChargingSystem {
private:
    // WebSocket Client setup
    //std::unique_ptr<ocpp::websockets::IWebsocketClient> wsClient;
    // You need to do this *after* initializing `wsClient`
    //rpcClient = std::make_unique<ocpp::rpc::RpcClient>(*wsClient, "ocpp1.6j");

    std::unique_ptr<ocpp::rpc::RpcClient> rpcClient;

    //std::string serverUrl = "ws://46.23.63.60:8080/steve/websocket/CentralSystemService/";  // Replace with your server's URL
    //auto wsClient = ocpp::websockets::WebsocketFactory::createWebSocketClient(serverUrl);

    //std::string stationID = "EVSE_01";  // Replace with your EVSE station ID

    // File paths
    const std::string state_file = "/tmp/charging_state.txt";
    const std::string pwm_config_file = "/tmp/pwm_config.txt";
    const std::string counter_file_1 = "/tmp/kwh_counter_1.txt";
    const std::string counter_file_2 = "/tmp/kwh_counter_2.txt";
    modbus_t* ctx = nullptr;
    // PWM constants
    const int PWM_FULL_SIGNAL = 255;
    int PWM_CHARGING_SIGNAL = 75;
    
    // PWM values for Green LED brightness adjustment
    const int MIN_GREEN_BRIGHTNESS = 50;
    const int MAX_GREEN_BRIGHTNESS = 255;

    // MCP3008 configuration
    #define MCP3008_CHANNEL_1 0
    #define MCP3008_CHANNEL_2 1
    #define MCP3008_CLK 11
    #define MCP3008_MISO 9
    #define MCP3008_MOSI 10
    #define MCP3008_CS 8

    // GPIO pins
    #define CP_OUT_1 12
    #define CP_OUT_2 13
    #define RELAY_1 16
    #define RELAY_2 17
    #define S0_PIN_1 4
    #define S0_PIN_2 5

    #define SPI_CHANNEL 0
    #define SPI_SPEED 100000

    // GPIO pins for RGB LED
    #define LED_RED_PIN 27
    #define LED_GREEN_PIN 22
    #define LED_BLUE_PIN 26

    #define DEVICE "/dev/ttyS0"  // RS-485 interface (USB to RS-485)
    #define SLAVE_ID 1             // Modbus slave ID for SMD630
    #define BAUD_RATE 9600         // Baud rate for Modbus RTU
    #define DATA_BITS 8            // Data bits (standard for Modbus)
    #define PARITY 'N'             // No parity
    #define STOP_BITS 1            // Stop bits (standard for Modbus)

    // Registers to read from the SMD630
    #define VOLTAGE_REGISTER 0
    #define CURRENT_REGISTER 6
    #define FREQUENCY_REGISTER 70
    #define POWER_REGISTER 12

    // Voltage thresholds
    const int VOLTAGE_STATE_A = 970;
    const int VOLTAGE_STATE_B = 860;
    const int VOLTAGE_STATE_C = 680;

    // Global variables
    std::string state_1 = "";
    std::string state_2 = "";
    int ocpp_counter = 0;
    int last_status_connector1 = -1;
    int last_status_connector2 = -1;
    std::string transactionIDConnector1 = "";
    std::string transactionIDConnector2 = "";

    // Charging counter
    int charging_counter = 0;
    bool permission_charging_connector1 = false;
    bool permission_charging_connector2 = false;
    int charging_1 = 0;
    int charging_2 = 0;
    int loops_in_state_f = 0;

    // Callback counters for S0 pins
    static int s0_counter_1;
    static int s0_counter_2;
  
    ChargingSystem(const ChargingSystem&) = delete;
    ChargingSystem& operator=(const ChargingSystem&) = delete;

public:

    //static ChargingSystem* instance;
     static ChargingSystem& instance() {
        static ChargingSystem instance;  // Guaranteed to be destroyed.
        return instance;                 // Instantiated on first use.
    }

    ChargingSystem() {
        signal(SIGINT, &ChargingSystem::cleanup);
    }

    // Utility function to get the current timestamp in ISO 8601 format
    std::string getCurrentTimestamp() {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::gmtime(&in_time_t), "%Y-%m-%dT%H:%M:%SZ");
        return ss.str();
    }

    // Connect to the server using WebSocket
   void connectToServer() {
    // Create WebSocket client using the new API
 
  wsClient = std::unique_ptr<ocpp::websockets::IWebsocketClient>(ocpp::websockets::WebsocketFactory::newClient());

  // Pass the arguments to the constructor
  //rpcClient = std::make_unique<ocpp::rpc::RpcClient>(*wsClient, protocol);

    if (!wsClient) {
        std::cerr << "Failed to create WebSocket client!" << std::endl;
        return;
    }
    const ocpp::websockets::IWebsocketClient::Credentials globalCreds{
    "admin", "@Carpoint1" 
    };

    //Credentials creds;
    //creds.useBasicAuth = true;
    //creds.username = "admin";
    //creds.password = "@Carpoint1";

    if (wsClient->connect(serverUrl, "", globalCreds, 5000ms, 10000ms)) {        
     std::cout << "WebSocket client connected!" << std::endl;
    } else {
        std::cerr << "Failed to connect WebSocket client." << std::endl;
    }
  }

    // Send energy readings to the server using WebSocket
    void sendEnergyReadingToServer(float voltage, float current, float power) {
        if (wsClient && wsClient->isConnected()) {
        Json::Value message;
        message["voltage"] = voltage;
        message["current"] = current;
        message["power"] = power;
	// Pass the arguments to the constructor
        rpcClient = std::make_unique<ocpp::rpc::RpcClient>(*wsClient, protocol);


	rpcClient->sendMessage(message.toStyledString());

	//rpcClient.sendMessage(message.toStyledString());

        //wsClient->sendMessage(message.toStyledString());
            }
    }

    // Load PWM configuration from file
    void load_pwm_config() {
        std::ifstream infile(pwm_config_file);
        if (infile.is_open()) {
            std::string line;
            while (std::getline(infile, line)) {
                size_t delimiter_pos = line.find('=');
                if (delimiter_pos == std::string::npos) continue;

                std::string key = line.substr(0, delimiter_pos);
                std::string value = line.substr(delimiter_pos + 1);

                if (key == "PWM_CHARGING_SIGNAL") {
                    int new_pwm_value = std::stoi(value);
                    if (new_pwm_value >= 0 && new_pwm_value <= 255 && PWM_CHARGING_SIGNAL != new_pwm_value) {
                        PWM_CHARGING_SIGNAL = new_pwm_value;
                        std::cout << "Updated PWM_CHARGING_SIGNAL to: " << PWM_CHARGING_SIGNAL << std::endl;
                    }
                }
            }
            infile.close();
        } else {
            PWM_CHARGING_SIGNAL = 75;  // Default fallback
        }
    }

    // Save charging state to file
    void save_charging_state() {
        std::ofstream state_out(state_file);
        if (state_out.is_open()) {
            state_out << "State 1: " << state_1 << "\n";
            state_out << "State 2: " << state_2 << "\n";
            state_out.close();
        } else {
            std::cerr << "Failed to write to charging state file: " << state_file << std::endl;
        }
    }

    // Read from MCP3008 ADC
    int read_mcp3008(int channel) {
        if (channel < 0 || channel > 7) {
            std::cerr << "Invalid channel: " << channel << std::endl;
            return -1;
        }

        unsigned char tx_buffer[3];  // Buffer to transmit
        unsigned char rx_buffer[3];  // Buffer to receive

        tx_buffer[0] = 0x01;  // Start bit for MCP3008
        tx_buffer[1] = 0x80 | (channel << 4);  // Control byte (start bit + channel select)
        tx_buffer[2] = 0x00;  // Dummy byte for reading the response

        // Open SPI device and perform read
        int spi_handle = spiOpen(SPI_CHANNEL, SPI_SPEED, 0);
        if (spi_handle < 0) {
            std::cerr << "Failed to open SPI!" << std::endl;
            return -1;
        }

        // Perform SPI transfer (send tx_buffer, receive to rx_buffer)
        int result = spiXfer(spi_handle, (char*)tx_buffer, (char*)rx_buffer, 3);
        spiClose(spi_handle);

        if (result < 0) {
            std::cerr << "SPI transfer failed!" << std::endl;
            return -1;
        }

        // Combine the result into a 10-bit value
        int adc_value = ((rx_buffer[1] & 0x03) << 8) | rx_buffer[2];
        return adc_value;
    }

    // Convert ADC to Voltage
    double convertADCToVolt(int adc) {
        return (4.93307e-6 * adc * adc) + (0.0182046 * adc) - 11.7888;
    }

    // Find peak voltage
    double find_peak_voltage(int channel) {
        double peak = 0;
        for (int i = 0; i < 500; i++) { // Perform multiple readings for better accuracy
            double current_voltage = read_mcp3008(channel);
            if (current_voltage > peak) {
                peak = current_voltage;
            }
            usleep(5); // Delay between readings
        }
        return peak;
    }


    // LED control: Set the color of the RGB LED based on the charging state
  void setLEDState(const std::string& state) {
	    signal(SIGINT, &ChargingSystem::cleanup);

        if (state == "error") {
            gpioWrite(LED_RED_PIN, 1);
            gpioWrite(LED_GREEN_PIN, 0);
            gpioWrite(LED_BLUE_PIN, 0);
        } else if (state == "blue") {
            gpioWrite(LED_RED_PIN, 0);
            gpioWrite(LED_GREEN_PIN, 0);
            gpioWrite(LED_BLUE_PIN, 1);
        } else if(state == "connected"){
            gpioWrite(LED_RED_PIN, 0);
            gpioWrite(LED_GREEN_PIN, 0);

            // Gradually control Blue LED brightness
            //while(true){
            for (int pwm_value = MAX_GREEN_BRIGHTNESS; pwm_value >= MIN_GREEN_BRIGHTNESS; pwm_value -= 10) {
                gpioPWM(LED_BLUE_PIN, pwm_value);
                usleep(50000);
            }
           for (int pwm_value = MIN_GREEN_BRIGHTNESS; pwm_value <= MAX_GREEN_BRIGHTNESS; pwm_value += 10) {
                gpioPWM(LED_BLUE_PIN, pwm_value);
                usleep(50000);
            }
        } else if (state == "charging") {
            gpioWrite(LED_RED_PIN, 0);
            gpioWrite(LED_BLUE_PIN, 0);

            // Gradually control Green LED brightness
            //while(true){
            for (int pwm_value = MAX_GREEN_BRIGHTNESS; pwm_value >= MIN_GREEN_BRIGHTNESS; pwm_value -= 5) {
                gpioPWM(LED_GREEN_PIN, pwm_value);
                usleep(50000);
            }
           for (int pwm_value = MIN_GREEN_BRIGHTNESS; pwm_value <= MAX_GREEN_BRIGHTNESS; pwm_value += 5) {
                gpioPWM(LED_GREEN_PIN, pwm_value);
                usleep(50000);
            }
            //}
        } else {
            gpioWrite(LED_RED_PIN, 0);
            gpioWrite(LED_GREEN_PIN, 0);
            gpioWrite(LED_BLUE_PIN, 0);
        }
    }


    // S0 pulse counter callback for kWh reading
    static void s0_callback_1(int gpio, int level, uint32_t tick) {
        if (level == 1) {
            ChargingSystem::instance().s0_counter_1++;
        }
    }

    static void s0_callback_2(int gpio, int level, uint32_t tick) {
        if (level == 1) {
            ChargingSystem::instance().s0_counter_2++;
        }
    }

    // Initialize the system
    int initialize() {
        if (gpioInitialise() < 0) {
            std::cerr << "Failed to initialize pigpio" << std::endl;
            return -1;
        }
        std::cout << "GPIO initialized successfully" << std::endl;
	//ChargingSystem::instance = this;

        // GPIO setup
        gpioSetMode(CP_OUT_1, PI_OUTPUT);
        gpioSetMode(CP_OUT_2, PI_OUTPUT);
        gpioSetMode(RELAY_1, PI_OUTPUT);
        gpioSetMode(RELAY_2, PI_OUTPUT);
        gpioSetMode(S0_PIN_1, PI_INPUT);
        gpioSetMode(S0_PIN_2, PI_INPUT);
        gpioSetPullUpDown(S0_PIN_1, PI_PUD_UP);
        gpioSetPullUpDown(S0_PIN_2, PI_PUD_UP);
        gpioWrite(RELAY_1, 0);
        gpioWrite(RELAY_2, 0);
        gpioSetPWMfrequency(CP_OUT_1, 1000);
        gpioSetPWMfrequency(CP_OUT_2, 1000);
        gpioPWM(CP_OUT_1, PWM_FULL_SIGNAL);
        gpioPWM(CP_OUT_2, PWM_FULL_SIGNAL);

        // Register S0 callbacks for pulse counting
        gpioSetISRFunc(S0_PIN_1, EITHER_EDGE, 100, s0_callback_1);
        gpioSetISRFunc(S0_PIN_2, EITHER_EDGE, 100, s0_callback_2);

        // Initialize LED pins
        gpioSetMode(LED_RED_PIN, PI_OUTPUT);
        gpioSetMode(LED_GREEN_PIN, PI_OUTPUT);
        gpioSetMode(LED_BLUE_PIN, PI_OUTPUT);

        return 0;
    }


    // Modbus connection setup for SMD630
    modbus_t* createModbusContext(int slave_id) {
        modbus_t* ctx = modbus_new_rtu(DEVICE, BAUD_RATE, PARITY, DATA_BITS, STOP_BITS);
        if (ctx == nullptr) {
            std::cerr << "Unable to create the libmodbus context" << std::endl;
            return nullptr;
        }
        modbus_set_slave(ctx, slave_id);
        if (modbus_connect(ctx) == -1) {
            std::cerr << "Unable to connect to the Modbus slave: " << modbus_strerror(errno) << std::endl;
            modbus_free(ctx);
            return nullptr;
        }
        std::cout << "ctx is: " << ctx;
        return ctx;
    }

    void cleanupModbus() {
        if (ctx) {
            modbus_close(ctx);
            modbus_free(ctx);
            ctx = nullptr;
        }
    }


    // Function to combine the two 16-bit registers and apply scaling
    float convertTofloat(uint16_t reg1, uint16_t reg2) {
        uint32_t combined = (static_cast<uint32_t>(reg1) << 16) | reg2;  // Combine registers in little-endian format
        float result;
        std::memcpy(&result, &combined, sizeof(result));
        return result; // Assuming the voltage is in tenths of a volt (check device manual)
    }

    void readSMD630Data(modbus_t* ctx) {
        uint16_t tab_reg[2];  // Buffer to store the values
        float voltage;
        float current;
        float power;

        // Read Voltage
        if (modbus_read_input_registers(ctx, VOLTAGE_REGISTER, 2, tab_reg) == -1) {
            std::cerr << "Failed to read voltage: " << modbus_strerror(errno) << std::endl;
        } else {
            std::cout << "Raw Voltage Registers: " << tab_reg[0] << " and " << tab_reg[1] << std::endl;
            voltage = convertTofloat(tab_reg[0], tab_reg[1]);
            std::cout << "Total Voltage: " << voltage << " V" << std::endl;
        }
    
        // Read Current
        if (modbus_read_input_registers(ctx, CURRENT_REGISTER, 2, tab_reg) == -1) {
            std::cerr << "Failed to read current: " << modbus_strerror(errno) << std::endl;
        } else {
            std::cout << "Raw Current Registers: " << tab_reg[0] << " and " << tab_reg[1] << std::endl;
            current = convertTofloat(tab_reg[0], tab_reg[1]);
            std::cout << "Total Current: " << current << " A" << std::endl;
        }
    
        // Read Frequency
        if (modbus_read_input_registers(ctx, FREQUENCY_REGISTER, 2, tab_reg) == -1) {
            std::cerr << "Failed to read frequency: " << modbus_strerror(errno) << std::endl;
        } else {
            std::cout << "Raw Frequency Registers: " << tab_reg[0] << " and " << tab_reg[1] << std::endl;
            float frequency = convertTofloat(tab_reg[0], tab_reg[1]);
            std::cout << "Frequency: " << frequency << " Hz" << std::endl;
        }
    
        // Read Power
        if (modbus_read_input_registers(ctx, POWER_REGISTER, 2, tab_reg) == -1) {
            std::cerr << "Failed to read power: " << modbus_strerror(errno) << std::endl;
        } else {
            std::cout << "Raw Power Registers: " << tab_reg[0] << " and " << tab_reg[1] << std::endl;
            power = convertTofloat(tab_reg[0], tab_reg[1]);
            std::cout << "Total Power: " << power << " W" << std::endl;
        }
	 sendEnergyReadingToServer(voltage, current, power);
    }


    // Charging loop
    void charging_loop() {
        try {
            while (true) {
                load_pwm_config();
                charging_counter++;
		
		// Create Modbus context to SMD630 (slave id = 1)
                modbus_t* ctx = createModbusContext(SLAVE_ID);
                if (ctx != nullptr) {
                // Read data from SMD630 (voltage, current, frequency, power)
		        std::cout << "Reading SMD630 datas...";
                readSMD630Data(ctx);
                // After reading data, close the connection
                
                }
                modbus_close(ctx);
                modbus_free(ctx);
		
		double peak_voltage_1 = find_peak_voltage(MCP3008_CHANNEL_1);
                double peak_voltage_2 = find_peak_voltage(MCP3008_CHANNEL_2);

                if (charging_counter > 10) {
		        //std::cout << "entering..." << std::endl;
                    charging_counter = 0;
                    save_charging_state();

                    // Log data
                    std::time_t now = std::time(nullptr);
                    char buffer[80];
                    struct tm *timeinfo = localtime(&now);
                    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
                    float kWh_1 = s0_counter_1 / 1000.0;
                    float kWh_2 = s0_counter_2 / 1000.0;
                    std::cout << "Timestamp: " << buffer << ", Channel 1 Peak Voltage: " << peak_voltage_1
                            << ", Channel 2 Peak Voltage: " << peak_voltage_2
                            << ", State 1: " << state_1 << ", State 2: " << state_2
                            << ", kWh 1: " << kWh_1 << ", kWh 2: " << kWh_2 << std::endl;
                }

                // Update charging states based on voltage
                if (peak_voltage_1 > VOLTAGE_STATE_A) {
                    loops_in_state_f = 0;
                    state_1 = 'a';
                    charging_1 = 0;
                    gpioPWM(CP_OUT_1, PWM_FULL_SIGNAL);
                    gpioWrite(RELAY_1, 0);
                    setLEDState("blue");
                } else if (peak_voltage_1 > VOLTAGE_STATE_B) {
                    loops_in_state_f = 0;
                    state_1 = 'b';
                    charging_1 = 0;
                    gpioPWM(CP_OUT_1, PWM_CHARGING_SIGNAL);
                    gpioWrite(RELAY_1, 0);
                    setLEDState("connected");
                    usleep(300000);
                } else if (peak_voltage_1 > VOLTAGE_STATE_C) {
                    loops_in_state_f = 0;
                    state_1 = 'c';
                    charging_1 = 1;
                    gpioPWM(CP_OUT_1, PWM_CHARGING_SIGNAL);
                    gpioWrite(RELAY_1, 1);
                    setLEDState("charging");
                    usleep(300000);
                } else {
                    loops_in_state_f++;
                    state_1 = 'f';
                    charging_1 = 0;
                    gpioPWM(CP_OUT_1, 0);
                    gpioWrite(RELAY_1, 0);
                    setLEDState("error");
                }

                if (loops_in_state_f > 5) {
                    gpioPWM(CP_OUT_1, PWM_FULL_SIGNAL);
                    gpioWrite(RELAY_1, 0);
                }

                // Handle channel 2 (same as channel 1 but for the second connector)
                if (peak_voltage_2 > VOLTAGE_STATE_A) {
                    state_2 = 'a';
                    charging_2 = 0;
                    gpioPWM(CP_OUT_2, PWM_FULL_SIGNAL);
                    gpioWrite(RELAY_2, 0);
                    setLEDState("blue");
                } else if (peak_voltage_2 > VOLTAGE_STATE_B) {
                    state_2 = 'b';
                    charging_2 = 0;
                    gpioPWM(CP_OUT_2, PWM_CHARGING_SIGNAL);
                    gpioWrite(RELAY_2, 0);
                    setLEDState("connected");
                } else if (peak_voltage_2 > VOLTAGE_STATE_C) {
                    state_2 = 'c';
                    charging_2 = 1;
                    gpioPWM(CP_OUT_2, PWM_CHARGING_SIGNAL);
                    gpioWrite(RELAY_2, 1);
                    setLEDState("charging");
                } else {
                    state_2 = 'f';
                    charging_2 = 0;
                    gpioPWM(CP_OUT_2, 0);
                    gpioWrite(RELAY_2, 0);
                    setLEDState("error");
                }
 
              
                // Simulate charging
                std::this_thread::sleep_for(std::chrono::seconds(5));  // Sleep for 5 seconds
            }
        } catch (const std::exception& e) {
            std::cerr << "Error in charging loop: " << e.what() << std::endl;
        }
    }

    // Cleanup function to terminate pigpio and WebSocket connection
    static void cleanup(int sig) {
        std::cerr << "Exiting program due to signal: " << sig << std::endl;

        // Stop the OCPP and charging loops
        gpioPWM(CP_OUT_1, 0);
        gpioPWM(CP_OUT_2, 0);
        gpioWrite(RELAY_1, 0);
        gpioWrite(RELAY_2, 0);

        // Terminate pigpio library
        gpioTerminate();

        // Exit the program
        exit(sig);
    }
};

int ChargingSystem::s0_counter_1 = -1;
int ChargingSystem::s0_counter_2 = -1;


int main() {
    ChargingSystem chargingSystem;

    // Initialize the system
    if (chargingSystem.initialize() != 0) {
        std::cerr << "Failed to initialize the system!" << std::endl;
        return -1;
    }

    // Connect to the WebSocket server
    chargingSystem.connectToServer();

    // Start charging loop in a separate thread
    std::thread chargingThread(&ChargingSystem::charging_loop, &chargingSystem);
    chargingThread.join();

    return 0;
}
