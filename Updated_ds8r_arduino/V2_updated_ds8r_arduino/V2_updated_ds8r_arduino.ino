#include <Arduino.h>

// Define pins for 4 DS8R devices
const int triggerPin0 = A0; // DS8R 0
const int triggerPin1 = A1; // DS8R 1
const int triggerPin2 = A2; // DS8R 2
const int triggerPin3 = A3; // DS8R 3

// Parameters for each device
float pulseFreq[4] = {25.0, 25.0, 25.0, 25.0};   // Hz
float duration[4] = {0.5, 0.5, 0.5, 0.5};        // seconds

// Stop flags for each device
volatile bool stopDevice[4] = {false, false, false, false};
unsigned long start_time[4] = {0, 0, 0, 0};
unsigned long end_time[4] = {0, 0, 0, 0};
unsigned long next_toggle_time[4] = {0, 0, 0, 0};
unsigned long half_pulse_time[4] = {0, 0, 0, 0};
int pulse_state[4] = {0, 0, 0, 0};

int device_count = 4;

long _previous_debug_pulse = -1;
long _debug_pulse_interval = 10000;

// Debug control (off by default to avoid flooding host serial)
volatile bool g_debug_enabled = false;

// Persistent input buffer (accumulate until CR/LF)
static char g_cmd_buffer[128];
static size_t g_cmd_len = 0;

void setup() 
{
    pinMode(triggerPin0, OUTPUT);
    pinMode(triggerPin1, OUTPUT);
    pinMode(triggerPin2, OUTPUT);
    pinMode(triggerPin3, OUTPUT);

    digitalWrite(triggerPin0, LOW);
    digitalWrite(triggerPin1, LOW);
    digitalWrite(triggerPin2, LOW);
    digitalWrite(triggerPin3, LOW);

    // Restore baud to match host manager defaults
    Serial.begin(9600);
    while (!Serial);

    if (g_debug_enabled) {
        Serial.println("[DEBUG] DS8R Arduino ready");
    }
}

void loop() 
{
    //Get the current millis
    unsigned long current_millis = millis();

    //Get the current micros
    unsigned long current_micros = micros();
    
    // Debugging pulse (DISABLED by default)
    // if ((_previous_debug_pulse == -1) || (current_millis >= (_previous_debug_pulse + _debug_pulse_interval))) {
    //     _previous_debug_pulse = current_millis;
    //     if (g_debug_enabled) {
    //         Serial.print("[DEBUG] Application debug pulse (millis = ");
    //         Serial.print(current_millis);
    //         Serial.println(")");
    //     }
    // }

    //Handle serial input
    handle_serial_input();

    //Handle the state of the stimulation pulse trains
    for (int i = 0; i < device_count; i++)
    {
        handle_stimulation_pulse_train_state(i, micros());
    }
}

void handle_serial_input ()
{
    // Accumulate bytes into line buffer; process on CR/LF
    while (Serial.available() > 0)
    {
        char c = Serial.read();

        // Treat CR and LF as terminators
        if (c == '\n' || c == '\r')
        {
            if (g_cmd_len == 0) {
                continue; // ignore empty lines
            }
            g_cmd_buffer[g_cmd_len] = '\0';
            g_cmd_len = 0;

            bool invalid_command_flag = false;

            // Debug control: DBG1/DBG0 (enable/disable verbose logs)
            if (startsWith(g_cmd_buffer, "DBG1")) {
                g_debug_enabled = true;
                Serial.println("[DEBUG] Debug logging enabled");
                continue;
            } else if (startsWith(g_cmd_buffer, "DBG0")) {
                g_debug_enabled = false;
                // Keep TX quiet
                continue;
            }

            char *command = g_cmd_buffer;

            if (startsWith(command, "STOP_ALL"))
            {
                for (int i = 0; i < device_count; i++)
                {
                    stopDevice[i] = true;
                    digitalWrite(getPinForDevice(i), LOW);
                }
                if (g_debug_enabled) Serial.println("[DEBUG] All devices stopped");
            }
            else if (startsWith(command, "STOP"))
            {
                if (strlen(command) >= 5 && isDigit(command[4]))
                {
                    int dev = command[4] - '0';
                    if (dev >= 0 && dev < device_count)
                    {
                        stopDevice[dev] = true;
                        digitalWrite(getPinForDevice(dev), LOW);
                        if (g_debug_enabled) {
                            Serial.print("[DEBUG] Stop command received for device ");
                            Serial.println(dev);
                        }
                    }
                    else
                    {
                        invalid_command_flag = true;
                    }
                }
                else
                {
                    invalid_command_flag = true;
                }
            }
            else if ((strlen(command) >= 2) && ((command[0] == 'T') || (command[0] == 'S')) && (isDigit(command[1])))
            {
                char command_char = command[0];
                int dev = command[1] - '0';
                
                if (dev >= 0 && dev < device_count)
                {
                    if (strlen(command) > 2)
                    {
                        if (command_char == 'T')
                        {
                            parseAndUpdateValues(command, dev);
                            initialize_pulse_train(dev);
                        }
                        else if (command_char == 'S')
                        {
                            parseAndUpdateValues(command, dev);
                            // 'S' only updates params, no trigger
                        }
                    }
                    else if (command_char == 'T')
                    {
                        // "T<dev>" triggers using existing params
                        initialize_pulse_train(dev);
                    }
                    else
                    {
                        invalid_command_flag = true;
                    }
                }
                else
                {
                    invalid_command_flag = true;
                }
            }
            else
            {
                invalid_command_flag = true;
            }

            if (invalid_command_flag)
            {
                // Keep error visible even if debug disabled
                Serial.println("[DEBUG] Invalid command.");
            }

            // Ready for next command
            continue;
        }

        // Ignore non-printable control chars
        if ((unsigned char)c < 0x20 && c != '\t') {
            continue;
        }

        // Append to buffer if space remains
        if (g_cmd_len < sizeof(g_cmd_buffer) - 1)
        {
            g_cmd_buffer[g_cmd_len++] = c;
        }
        else
        {
            // Overflow: reset buffer to avoid parsing garbage
            g_cmd_len = 0;
            Serial.println("[DEBUG] Invalid command.");
        }
    }
}

bool startsWith(const char *str, const char *pre)
{
    size_t lenpre = strlen(pre),
           lenstr = strlen(str);
    return lenstr < lenpre ? false : memcmp(pre, str, lenpre) == 0;
}

int getPinForDevice(int dev) 
{
    switch (dev) 
    {
        case 0: return triggerPin0;
        case 1: return triggerPin1;
        case 2: return triggerPin2;
        case 3: return triggerPin3;
        default: return -1;
    }
}

void parseAndUpdateValues(char *command, int dev) 
{
    char *token = strtok(command, ",");
    int i = 0;
    float new_freq = 0;
    float new_duration = 0;
    while (token != NULL)
    {
        if (i == 1)
        {
            new_freq = atof(token);
        }
        else if (i == 2)
        {
            new_duration = atof(token);
        }

        i++;
        token = strtok(NULL, ",");
    }

    if (new_freq > 0 && new_duration > 0)
    {
        pulseFreq[dev] = new_freq;
        duration[dev] = new_duration;
        if (g_debug_enabled) {
            Serial.print("[DEBUG] Updated params dev ");
            Serial.print(dev);
            Serial.print(" freq=");
            Serial.print(new_freq);
            Serial.print("Hz, dur=");
            Serial.print(new_duration);
            Serial.println("s");
        }
    }
    else if (i < 2)
    {
        Serial.println("[DEBUG] Invalid command format.");
    }
    else
    {
        Serial.println("[DEBUG] Invalid values.");
    }
}

void handle_stimulation_pulse_train_state (int dev, unsigned long current_micros)
{
    //Make sure dev is within the correct bounds
    if (dev < 0 || dev >= device_count)
    {
        return;
    }

    //Get the pin for this device
    int pin = getPinForDevice(dev);
    if (pin == -1)
    {
        return;
    }

    //If the "stop device" flag is set...
    if (stopDevice[dev] || ((end_time[dev] > 0) && (current_micros >= end_time[dev])))
    {
        if (end_time[dev] > 0 && g_debug_enabled)
        {
            unsigned long end_micros = micros();
            Serial.print("[DEBUG] Pulse train complete on device ");
            Serial.print(dev);
            Serial.print(". Duration: ");
            Serial.print((end_micros - start_time[dev]) / 1000.0);
            Serial.println(" ms");
        }
        //Reset all the timing info for this device
        start_time[dev] = 0;
        end_time[dev] = 0;
        half_pulse_time[dev] = 0;
        next_toggle_time[dev] = 0;

        //Make sure the pin is set to low when stopping
        digitalWrite(pin, LOW);

        //Clear the stop flag
        stopDevice[dev] = false;

        return;
    }

    if ((next_toggle_time[dev] > 0) && (current_micros >= next_toggle_time[dev]))
    {
        pulse_state[dev] = !pulse_state[dev];
        next_toggle_time[dev] = current_micros + half_pulse_time[dev];
        digitalWrite(pin, pulse_state[dev]);

        if (g_debug_enabled) {
            Serial.print("[DEBUG] Device ");
            Serial.print(dev);
            Serial.print(" pulse state: ");
            Serial.println(pulse_state[dev]);
        }
    }
}

void initialize_pulse_train (int dev)
{
    //Make sure dev is within the correct bounds
    if (dev < 0 || dev >= device_count)
    {
        return;
    }

    //Get the pin for this device
    int pin = getPinForDevice(dev);
    if (pin == -1)
    {
        return;
    }

    //Clear the stop flag for this device
    stopDevice[dev] = false;

    unsigned long pulsePeriod = 1000000UL / pulseFreq[dev];
    unsigned long halfPulse = pulsePeriod / 2;
    unsigned long startTime = micros();
    
    start_time[dev] = startTime;
    end_time[dev] = startTime + (unsigned long)(duration[dev] * 1e6);
    next_toggle_time[dev] = startTime;
    half_pulse_time[dev] = halfPulse;
    pulse_state[dev] = 0;

    if (g_debug_enabled) {
        Serial.print("[DEBUG] Starting pulse train on device ");
        Serial.println(dev);
    }
}

