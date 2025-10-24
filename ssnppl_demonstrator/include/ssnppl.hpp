// ****************************************************************************
//
// Copyright (c) 2023, Septentrio
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// ****************************************************************************

#ifndef __GLUECODE__
#define __GLUECODE__

#include "program_option.hpp"
#include "mqtt.hpp"
#include "SerialComm.hpp"
#include <thread>
#include <queue>
#include "PPL_PublicInterface.h" // PointPerfect Library
#include <vector>
#include <condition_variable>
#include <atomic>

enum ssnppl_error
{
    SUCCESS,
    FAIL,
    MQTT_ERROR,
    PPL_FAILED,
};

class Ssnppl_demonstrator
{
    
private:
    // State struct
    ProgramOptions options;
    char *currentDynKey;

    std::string freqInfo = "";
    std::string keyInfo = "";

    std::atomic<bool> update_receiver{false};
    std::atomic<bool> thread_running{true};


    ssnppl_error init_option(int argc, char *argv[]);
    void init_receiver();
    ssnppl_error init_ppl();

    // Serial Port
    SerialPort main_channel;
    SerialPort lband_channel;
    SerialPort spartn_input_channel;

    // Ephemeris GGA thread
    void read_ephemeris_gga_data();
    std::thread read_ephemeris_gga_data_thread;
    std::queue<std::vector<uint8_t>> ephemeris_gga_queue;
    std::mutex ephemeris_gga_mutex;

    // LBand thread
    void read_lband_data();
    std::thread read_lband_data_thread;
    std::queue<std::vector<uint8_t>> lband_queue;
    std::mutex lband_queue_mutex;

    // SPARTN Serial input thread
    void read_spartn_input_data();
    std::thread read_spartn_input_data_thread;
    std::queue<std::vector<uint8_t>> spartn_input_queue;
    std::mutex spartn_input_queue_mutex;

    // PPL thread
    void handle_data();

    // Send RTCM thread
    void write_rtcm();
    std::thread write_rtcm_thread;
    std::queue<std::vector<uint8_t>> rtcm_queue;
    std::mutex rtcm_queue_mutex;
    std::condition_variable cv_rtcm;

    std::condition_variable_any cv_incoming_data;
    std::mutex lk_incoming_data;

    ssnppl_error init_main_comm();
    ssnppl_error init_lband_comm();
    ssnppl_error init_serial();

    // MQTT
    struct mosquitto *mosq_client = nullptr;
    UserData userData;
    ssnppl_error init_mqtt();
    ssnppl_error switch_mqtt_server(std::string new_mqtt_server);

    // SPARTN LOG
    std::ofstream SPARTN_file_Ip;
    std::ofstream SPARTN_file_Lb;
    void init_SPARTN_LOG();

    // Localized Service
    std::vector<std::string> tile_dict;
    int tile_level{2};
    float latitude_threshold{0}; 
    float longitude_threshold{0};
    float latitude{0};
    float longitude{0};
    std::string nodeprefix ; 

    void process_new_position () noexcept;
    void process_new_node() noexcept ;
    std::string new_Node_Topic () noexcept;
    std::string new_Tile_Topic () noexcept;

    
public:
    // Default Ctor and Dtor
    Ssnppl_demonstrator() = default;
    virtual ~Ssnppl_demonstrator();

    // Init func
    ssnppl_error init(int argc, char *argv[]);
    ssnppl_error dispatch();
};

#endif

/*
 - add function to found new tile topic
 - add function to found new node topic
*/
