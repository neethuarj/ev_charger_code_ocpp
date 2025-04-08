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

// Add these headers for OCPP implementation
#include <websocketpp/config/asio_client.hpp>
#include <websocketpp/client.hpp>
#include <json/json.h>  // You're already using JSON
#include <chrono>
#include <map>

class OCPPClient {
    public:
        // WebSocket client typedefs
        typedef websocketpp::client<websocketpp::config::asio_tls_client> WSClient;
        typedef websocketpp::config::asio_tls_client::message_type::ptr message_ptr;
        typedef websocketpp::lib::shared_ptr<websocketpp::lib::asio::ssl::context> context_ptr;
    
        // OCPP message types
        enum MessageType {
            CALL = 2,
            CALLRESULT = 3,
            CALLERROR = 4
        };
    
        OCPPClient(const std::string& chargePointId, const std::string& serverURL) 
            : m_chargePointId(chargePointId), m_serverURL(serverURL), m_connected(false), m_messageCounter(0) {
            
            // Initialize WebSocket client
            m_client.clear_access_channels(websocketpp::log::alevel::all);
            m_client.set_access_channels(websocketpp::log::alevel::connect);
            m_client.set_access_channels(websocketpp::log::alevel::disconnect);
            m_client.set_access_channels(websocketpp::log::alevel::app);
    
            // Initialize ASIO
            m_client.init_asio();
    
            // Setup TLS handling
            m_client.set_tls_init_handler([](websocketpp::connection_hdl) {
                context_ptr ctx = websocketpp::lib::make_shared<boost::asio::ssl::context>(boost::asio::ssl::context::tlsv12);
                try {
                    ctx->set_options(boost::asio::ssl::context::default_workarounds |
                                    boost::asio::ssl::context::no_sslv2 |
                                    boost::asio::ssl::context::no_sslv3 |
                                    boost::asio::ssl::context::single_dh_use);
                } catch (std::exception& e) {
                    std::cout << "Error in TLS setup: " << e.what() << std::endl;
                }
                return ctx;
            });
    
            // Setup message handlers
            m_client.set_open_handler(std::bind(&OCPPClient::onOpen, this, std::placeholders::_1));
            m_client.set_close_handler(std::bind(&OCPPClient::onClose, this, std::placeholders::_1));
            m_client.set_fail_handler(std::bind(&OCPPClient::onFail, this, std::placeholders::_1));
            m_client.set_message_handler(std::bind(&OCPPClient::onMessage, this, std::placeholders::_1, std::placeholders::_2));
        }
    
        // Connect to OCPP server
        bool connect() {
            try {
                websocketpp::lib::error_code ec;
                WSClient::connection_ptr con = m_client.get_connection(m_serverURL, ec);
                if (ec) {
                    std::cout << "Connect initialization error: " << ec.message() << std::endl;
                    return false;
                }
    
                // Store connection
                m_hdl = con->get_handle();
                m_client.connect(con);
    
                // Start ASIO thread
                m_thread = std::thread(&WSClient::run, &m_client);
                return true;
            } catch (const std::exception& e) {
                std::cout << "Exception in connect: " << e.what() << std::endl;
                return false;
            }
        }
    
        // Disconnect from OCPP server
        void disconnect() {
            if (m_connected) {
                try {
                    m_client.close(m_hdl, websocketpp::close::status::normal, "Closing connection");
                    m_connected = false;
                } catch (const std::exception& e) {
                    std::cout << "Exception in disconnect: " << e.what() << std::endl;
                }
            }
    
            if (m_thread.joinable()) {
                m_client.stop();
                m_thread.join();
            }
        }
    
        // Send Boot Notification
        std::string sendBootNotification() {
            Json::Value payload;
            payload["chargePointVendor"] = "YourVendor";
            payload["chargePointModel"] = "YourModel";
            payload["chargePointSerialNumber"] = "YourSerialNumber";
            payload["firmwareVersion"] = "1.0.0";
            
            return sendOCPPMessage(MessageType::CALL, generateMessageId(), "BootNotification", payload);
        }
    
        // Send Heartbeat
        std::string sendHeartbeat() {
            Json::Value payload;  // Empty payload
            return sendOCPPMessage(MessageType::CALL, generateMessageId(), "Heartbeat", payload);
        }
    
        // Send Status Notification
        std::string sendStatusNotification(int connectorId, const std::string& status, const std::string& errorCode = "NoError") {
            Json::Value payload;
            payload["connectorId"] = connectorId;
            payload["errorCode"] = errorCode;
            payload["status"] = status;
            payload["timestamp"] = getTimestamp();
            
            return sendOCPPMessage(MessageType::CALL, generateMessageId(), "StatusNotification", payload);
        }
    
        // Start Transaction
        std::string startTransaction(int connectorId, const std::string& idTag, int meterStart) {
            Json::Value payload;
            payload["connectorId"] = connectorId;
            payload["idTag"] = idTag;
            payload["timestamp"] = getTimestamp();
            payload["meterStart"] = meterStart;
            
            return sendOCPPMessage(MessageType::CALL, generateMessageId(), "StartTransaction", payload);
        }
    
        // Stop Transaction
        std::string stopTransaction(int transactionId, const std::string& idTag, int meterStop, const std::string& reason = "Local") {
            Json::Value payload;
            payload["transactionId"] = transactionId;
            payload["idTag"] = idTag;
            payload["timestamp"] = getTimestamp();
            payload["meterStop"] = meterStop;
            payload["reason"] = reason;
            
            return sendOCPPMessage(MessageType::CALL, generateMessageId(), "StopTransaction", payload);
        }
    
        // Check if connected
        bool isConnected() const {
            return m_connected;
        }
    
        // Create a map to store received transaction IDs
        std::map<std::string, std::string> transactionIdMap;
    
    private:
        WSClient m_client;
        websocketpp::connection_hdl m_hdl;
        std::thread m_thread;
        std::string m_chargePointId;
        std::string m_serverURL;
        bool m_connected;
        int m_messageCounter;
        std::mutex m_mutex;
    
        // WebSocket event handlers
        void onOpen(websocketpp::connection_hdl hdl) {
            m_connected = true;
            std::cout << "Connection established with OCPP server" << std::endl;
            sendBootNotification();  // Send boot notification when connected
        }
    
        void onClose(websocketpp::connection_hdl hdl) {
            m_connected = false;
            std::cout << "Connection closed" << std::endl;
        }
    
        void onFail(websocketpp::connection_hdl hdl) {
            m_connected = false;
            std::cout << "Connection failed" << std::endl;
        }
    
        void onMessage(websocketpp::connection_hdl hdl, message_ptr msg) {
            std::cout << "Received message: " << msg->get_payload() << std::endl;
            processOCPPMessage(msg->get_payload());
        }
    
        // Process incoming OCPP messages
        void processOCPPMessage(const std::string& message) {
            Json::Value root;
            Json::Reader reader;
            
            if (!reader.parse(message, root)) {
                std::cout << "Failed to parse message" << std::endl;
                return;
            }
            
            int messageType = root[0].asInt();
            std::string messageId = root[1].asString();
            
            switch (messageType) {
                case MessageType::CALL: {
                    // Handle incoming request
                    std::string action = root[2].asString();
                    Json::Value payload = root[3];
                    handleIncomingRequest(messageId, action, payload);
                    break;
                }
                case MessageType::CALLRESULT: {
                    // Handle response to our request
                    Json::Value payload = root[2];
                    handleCallResult(messageId, payload);
                    break;
                }
                case MessageType::CALLERROR: {
                    // Handle error response
                    std::cout << "Received error for message ID: " << messageId << std::endl;
                    break;
                }
                default:
                    std::cout << "Unknown message type: " << messageType << std::endl;
                    break;
            }
        }
    
        // Handle incoming requests from Central System
        void handleIncomingRequest(const std::string& messageId, const std::string& action, const Json::Value& payload) {
            Json::Value response;
    
            if (action == "Reset") {
                // Handle Reset command
                response["status"] = "Accepted";
                sendOCPPMessage(MessageType::CALLRESULT, messageId, "", response);
                // Actual reset logic would go here
            }
            else if (action == "RemoteStartTransaction") {
                // Handle remote start
                int connectorId = payload["connectorId"].asInt();
                std::string idTag = payload["idTag"].asString();
                response["status"] = "Accepted";
                sendOCPPMessage(MessageType::CALLRESULT, messageId, "", response);
                // Actual start logic would go here
            }
            else if (action == "RemoteStopTransaction") {
                // Handle remote stop
                int transactionId = payload["transactionId"].asInt();
                response["status"] = "Accepted";
                sendOCPPMessage(MessageType::CALLRESULT, messageId, "", response);
                // Actual stop logic would go here
            }
            else {
                // Other actions
                response["status"] = "NotImplemented";
                sendOCPPMessage(MessageType::CALLRESULT, messageId, "", response);
            }
        }
    
        // Handle responses to our requests
        void handleCallResult(const std::string& messageId, const Json::Value& payload) {
            // Handle different responses based on what we sent
            if (payload.isMember("transactionId")) {
                // This is likely a response to StartTransaction
                std::string transactionId = payload["transactionId"].asString();
                // Store this transaction ID for later use
                transactionIdMap[messageId] = transactionId;
                std::cout << "Received transaction ID: " << transactionId << " for message: " << messageId << std::endl;
            }
        }
    
        // Send OCPP message
        std::string sendOCPPMessage(MessageType type, const std::string& id, const std::string& action, const Json::Value& payload) {
            Json::Value message(Json::arrayValue);
            message.append(type);
            message.append(id);
            
            if (type == MessageType::CALL) {
                message.append(action);
                message.append(payload);
            } else {
                message.append(payload);
            }
            
            Json::FastWriter writer;
            std::string msg = writer.write(message);
            // Remove trailing newline
            if (!msg.empty() && msg[msg.length()-1] == '\n') {
                msg.erase(msg.length()-1);
            }
            
            try {
                m_client.send(m_hdl, msg, websocketpp::frame::opcode::text);
                std::cout << "Sent message: " << msg << std::endl;
            } catch (const std::exception& e) {
                std::cout << "Error sending message: " << e.what() << std::endl;
            }
            
            return id;
        }
    
        // Generate unique message ID
        std::string generateMessageId() {
            std::lock_guard<std::mutex> lock(m_mutex);
            std::string id = m_chargePointId + "-" + std::to_string(++m_messageCounter);
            return id;
        }
    
        // Get ISO 8601 timestamp
        std::string getTimestamp() {
            auto now = std::chrono::system_clock::now();
            auto in_time_t = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << std::put_time(std::gmtime(&in_time_t), "%Y-%m-%dT%H:%M:%SZ");
            return ss.str();
        }
    };

    class ChargingSystem {
        
    private:
         // File paths
    const std::string state_file = "/tmp/charging_state.txt";
    const std::string pwm_config_file = "/tmp/pwm_config.txt";
    const std::string counter_file_1 = "/tmp/kwh_counter_1.txt";
    const std::string counter_file_2 = "/tmp/kwh_counter_2.txt";

    // FAILBACK
    int PWM_CHARGING_SIGNAL_FAILBACK = 68;

    // PWM constants
    const int PWM_FULL_SIGNAL = 255;
    int PWM_CHARGING_SIGNAL = 75;

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

    // PWM values for Green LED brightness adjustment
    const int MIN_GREEN_BRIGHTNESS = 50;
    const int MAX_GREEN_BRIGHTNESS = 255;

    // Voltage thresholds
    const int VOLTAGE_STATE_A = 970;
    const int VOLTAGE_STATE_B = 860;
    const int VOLTAGE_STATE_C = 680;

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

    #define GPIO_RS485_RX 14  // GPIO Pin to control direction (transmit/receive)
    #define GPIO_RS485_TX 15  // GPIO Pin to control direction (transmit/receive)
    modbus_t* ctx = nullptr;

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
    int s0_counter_1 = -1;
    int s0_counter_2 = -1;

        
        // Add these new members:
        std::unique_ptr<OCPPClient> ocppClient;
        std::thread heartbeatThread;
        std::atomic<bool> heartbeatRunning;
        std::string ocppServerURL;
        std::string chargePointId;
        
        // Status mapping from your states to OCPP states
        std::map<char, std::string> stateToOCPPStatus = {
            {'a', "Available"},
            {'b', "Preparing"},
            {'c', "Charging"},
            {'f', "Faulted"}
        };
    
    public:
        // Add this to your constructor
        ChargingSystem() : heartbeatRunning(false) {
            signal(SIGINT, &ChargingSystem::cleanup);
            ctx = createModbusContext(SLAVE_ID);
            
            // Set OCPP parameters (adjust these as needed)
            chargePointId = "CP001";
            ocppServerURL = "wss://your-ocpp-server.com";
        }
        
        // Add this to your destructor
        ~ChargingSystem() {
            stopOCPP();
            cleanupModbus();
        }


        // Utility function to get the current timestamp in ISO 8601 format
    std::string getCurrentTimestamp() {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::gmtime(&in_time_t), "%Y-%m-%dT%H:%M:%SZ");
        return ss.str();
    }

    // MCP3008 initialization
    void mcp3008_init() {
        gpioSetMode(MCP3008_CLK, PI_OUTPUT);
        gpioSetMode(MCP3008_MISO, PI_INPUT);
        gpioSetMode(MCP3008_MOSI, PI_OUTPUT);
        gpioSetMode(MCP3008_CS, PI_OUTPUT);

        gpioWrite(MCP3008_CS, 1);  // Ensure chip is deselected initially
        gpioWrite(MCP3008_CLK, 0); // Set clock to low
        std::cout << "MCP3008 initialized successfully" << std::endl;
    }

     // Load PWM configuration
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
            PWM_CHARGING_SIGNAL = PWM_CHARGING_SIGNAL_FAILBACK;
        }
    }

    // Load counter values
    void load_counter() {
        std::ifstream infile1(counter_file_1);
        if (infile1.is_open()) {
            infile1 >> s0_counter_1;
            infile1.close();
            std::cout << "Loaded s0_counter_1: " << s0_counter_1 << std::endl;
        } else {
            std::cerr << "Failed to open counter file 1: " << counter_file_1 << std::endl;
        }

        std::ifstream infile2(counter_file_2);
        if (infile2.is_open()) {
            infile2 >> s0_counter_2;
            infile2.close();
            std::cout << "Loaded s0_counter_2: " << s0_counter_2 << std::endl;
        } else {
            std::cerr << "Failed to open counter file 2: " << counter_file_2 << std::endl;
        }
    }

    // Save counter values
    void save_counter() {
        std::ofstream outfile1(counter_file_1);
        if (outfile1.is_open()) {
            outfile1 << s0_counter_1;
            outfile1.close();
        }

        std::ofstream outfile2(counter_file_2);
        if (outfile2.is_open()) {
            outfile2 << s0_counter_2;
            outfile2.close();
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

    // Save charging state
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

    // Cleanup function
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
// Initialize the system
int initialize() {
    if (gpioInitialise() < 0) {
        std::cerr << "Failed to initialize pigpio" << std::endl;
        return -1;
    }
    std::cout << "GPIO initialized successfully" << std::endl;

    // MCP3008 and GPIO setup
    mcp3008_init();
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

    return 0;
}

void setLEDState(const std::string& state) {
//signal(SIGINT, &ChargingSystem::cleanup);

    if (state == "error") {
        gpioWrite(LED_RED_PIN, 1);
        gpioWrite(LED_GREEN_PIN, 0);
        gpioWrite(LED_BLUE_PIN, 0);
    } else if (state == "blue") {
        gpioWrite(LED_RED_PIN, 0);
        gpioWrite(LED_GREEN_PIN, 0);
        gpioWrite(LED_BLUE_PIN, 1);
    } else if (state == "charging") {
        gpioWrite(LED_RED_PIN, 0);
        gpioWrite(LED_BLUE_PIN, 0);

        // Gradually reduce Green LED brightness
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
    
        // Read Voltage
        if (modbus_read_input_registers(ctx, VOLTAGE_REGISTER, 2, tab_reg) == -1) {
            std::cerr << "Failed to read voltage: " << modbus_strerror(errno) << std::endl;
        } else {
            std::cout << "Raw Voltage Registers: " << tab_reg[0] << " and " << tab_reg[1] << std::endl;
            float voltage = convertTofloat(tab_reg[0], tab_reg[1]);
            std::cout << "Total Voltage: " << voltage << " V" << std::endl;
        }
    
        // Read Current
        if (modbus_read_input_registers(ctx, CURRENT_REGISTER, 2, tab_reg) == -1) {
            std::cerr << "Failed to read current: " << modbus_strerror(errno) << std::endl;
        } else {
            std::cout << "Raw Current Registers: " << tab_reg[0] << " and " << tab_reg[1] << std::endl;
            float current = convertTofloat(tab_reg[0], tab_reg[1]);
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
            float power = convertTofloat(tab_reg[0], tab_reg[1]);
            std::cout << "Total Power: " << power << " W" << std::endl;
        }
    }
        
        // Initialize OCPP
        void initializeOCPP() {
            ocppClient = std::make_unique<OCPPClient>(chargePointId, ocppServerURL);
            if (ocppClient->connect()) {
                std::cout << "OCPP client connected successfully" << std::endl;
                
                // Start heartbeat thread
                heartbeatRunning = true;
                heartbeatThread = std::thread(&ChargingSystem::heartbeatLoop, this);
            } else {
                std::cerr << "Failed to connect OCPP client" << std::endl;
            }
        }
        
        // Stop OCPP
        void stopOCPP() {
            heartbeatRunning = false;
            if (heartbeatThread.joinable()) {
                heartbeatThread.join();
            }
            
            if (ocppClient) {
                ocppClient->disconnect();
            }
        }
        
        // Heartbeat loop
        void heartbeatLoop() {
            while (heartbeatRunning) {
                if (ocppClient && ocppClient->isConnected()) {
                    ocppClient->sendHeartbeat();
                }
                // Send heartbeat every 5 minutes
                std::this_thread::sleep_for(std::chrono::seconds(300));
            }
        }
        
        // Convert your internal state to OCPP status
        std::string mapStateToOCPPStatus(char state) {
            auto it = stateToOCPPStatus.find(state);
            if (it != stateToOCPPStatus.end()) {
                return it->second;
            }
            return "Unavailable";
        }
        
        // Update OCPP status based on charger state changes
        void updateOCPPStatus() {
            static char last_state_1 = 0;
            static char last_state_2 = 0;
            
            // Only send status notifications when state changes
            if (state_1[0] != last_state_1) {
                if (ocppClient && ocppClient->isConnected()) {
                    std::string status = mapStateToOCPPStatus(state_1[0]);
                    ocppClient->sendStatusNotification(1, status);
                }
                last_state_1 = state_1[0];
            }
            
            if (state_2[0] != last_state_2) {
                if (ocppClient && ocppClient->isConnected()) {
                    std::string status = mapStateToOCPPStatus(state_2[0]);
                    ocppClient->sendStatusNotification(2, status);
                }
                last_state_2 = state_2[0];
            }
        }
        
        // Handle transaction start
        void startOCPPTransaction(int connectorId) {
            if (!ocppClient || !ocppClient->isConnected()) return;
            
            std::string idTag = "DefaultTag"; // In a real system, this would come from RFID or app
            int meterValue = (connectorId == 1) ? s0_counter_1 : s0_counter_2;
            
            std::string msgId = ocppClient->startTransaction(connectorId, idTag, meterValue);
            
            // Store the message ID to match it with the transaction ID when response arrives
            if (connectorId == 1) {
                transactionIDConnector1 = msgId;
            } else if (connectorId == 2) {
                transactionIDConnector2 = msgId;
            }
        }
        
        // Handle transaction stop
        void stopOCPPTransaction(int connectorId) {
            if (!ocppClient || !ocppClient->isConnected()) return;
            
            std::string idTag = "DefaultTag"; // Should match the one used in startTransaction
            int meterValue = (connectorId == 1) ? s0_counter_1 : s0_counter_2;
            
            // Get the transaction ID from the map
            std::string msgId = (connectorId == 1) ? transactionIDConnector1 : transactionIDConnector2;
            auto it = ocppClient->transactionIdMap.find(msgId);
            
            if (it != ocppClient->transactionIdMap.end()) {
                int transactionId = std::stoi(it->second);
                ocppClient->stopTransaction(transactionId, idTag, meterValue);
                
                // Clear the transaction ID
                if (connectorId == 1) {
                    transactionIDConnector1 = "";
                } else if (connectorId == 2) {
                    transactionIDConnector2 = "";
                }
            }
        }
        
        // Modified charging loop to incorporate OCPP
        void charging_loop() {
            try {
                load_counter();
                
                // Initialize OCPP
                initializeOCPP();
                
                while (true) {
                    load_pwm_config();
                    charging_counter++;
                    std::cout << "charging counter value is: " << charging_counter;
                    modbus_t* ctx = createModbusContext(SLAVE_ID);
                    if (ctx != nullptr) {
                    // Read data from SMD630 (voltage, current, frequency, power)
                    std::cout << "Reading SMD630 datas...";
                    readSMD630Data(ctx);
                    //writeSMD630Data(ctx);
                    // After reading data, close the connection
                    
                    }
                    modbus_close(ctx);
                    modbus_free(ctx);
                    
                    double peak_voltage_1 = find_peak_voltage(MCP3008_CHANNEL_1);
                    double peak_voltage_2 = find_peak_voltage(MCP3008_CHANNEL_2);
                    
                    // Remember previous charging states to detect transitions
                    bool was_charging_1 = (state_1 == "c");
                    bool was_charging_2 = (state_2 == "c");
                    
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
                                setLEDState("error");
                            } else if (peak_voltage_1 > VOLTAGE_STATE_B) {
                                loops_in_state_f = 0;
                                state_1 = 'b';
                                charging_1 = 0;
                                gpioPWM(CP_OUT_1, PWM_CHARGING_SIGNAL);
                                gpioWrite(RELAY_1, 0);
                                setLEDState("blue");
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
                            } else if (peak_voltage_2 > VOLTAGE_STATE_B) {
                                state_2 = 'b';
                                charging_2 = 0;
                                gpioPWM(CP_OUT_2, PWM_CHARGING_SIGNAL);
                                gpioWrite(RELAY_2, 0);
                            } else if (peak_voltage_2 > VOLTAGE_STATE_C) {
                                state_2 = 'c';
                                charging_2 = 1;
                                gpioPWM(CP_OUT_2, PWM_CHARGING_SIGNAL);
                                gpioWrite(RELAY_2, 1);
                            } else {
                                state_2 = 'f';
                                charging_2 = 0;
                                gpioPWM(CP_OUT_2, 0);
                                gpioWrite(RELAY_2, 0);
                            }
                    // Add OCPP status updates
                    updateOCPPStatus();
                    
                    // Check for charging state transitions for connector 1
                    if (!was_charging_1 && state_1 == "c") {
                        // Transition from not charging to charging - start transaction
                        startOCPPTransaction(1);
                    } else if (was_charging_1 && state_1 != "c") {
                        // Transition from charging to not charging - stop transaction
                        stopOCPPTransaction(1);
                    }
                    
                    // Check for charging state transitions for connector 2
                    if (!was_charging_2 && state_2 == "c") {
                        // Transition from not charging to charging - start transaction
                        startOCPPTransaction(2);
                    } else if (was_charging_2 && state_2 != "c") {
                        // Transition from charging to not charging - stop transaction
                        stopOCPPTransaction(2);
                    }
                    
                    //usleep(100000); 
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Adjust the loop delay as needed
                }
            } catch (const std::exception& e) {
                std::cerr << "Error in charging loop: " << e.what() << std::endl;
                stopOCPP();
                cleanup(SIGINT); // Call cleanup in case of error
            }
        }
        
        // Modified cleanup method
        static void cleanup(int sig) {
            std::cerr << "Exiting program due to signal: " << sig << std::endl;
            
            // Get the instance (you may need to use a global pointer to your instance)
            ChargingSystem* instance = /* get instance */;
            if (instance) {
                instance->stopOCPP();
            }
            
            // Stop the charging loops
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

    int main() {
        ChargingSystem charging_system;
    
        if (charging_system.initialize() != 0) {
            return -1;
        }
    
        std::thread chargingThread(&ChargingSystem::charging_loop, &charging_system);
    
        // Join threads
        chargingThread.join();
    
        return 0;
    }

