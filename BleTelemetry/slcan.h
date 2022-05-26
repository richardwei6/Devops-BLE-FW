#ifndef SLCAN_H_
#define SLCAN_H_

#include <mbed.h>

class SLCANBase {
public:
    bool update();
    virtual ~SLCANBase();

//protected: #todo fix, proper struct
    SLCANBase();

    //To be implemented by subclasses
    //abstract functions
    virtual bool setBaudrate(int baudrate){}
    virtual bool setMode(CAN::Mode mode){}
    virtual bool transmitMessage(const CANMessage& msg){}

    virtual bool processCommands(){}
    virtual bool processCANMessages(){}
    virtual bool flush(){}
    virtual bool inputReadable(){}
    virtual int  readInputByte(){}

    virtual uint8_t getFirmwareVersion();
    virtual uint8_t getHardwareVersion();
    virtual const char* getSerialString();

    // Shared amongst subclasses
    static size_t formatCANMessage(const CANMessage& msg, char* buf, size_t max_len);
    static size_t formattedCANMessageLength(const CANMessage& msg);
    static size_t commandResponseLength(const char* command);
    bool execCommand(const char* command, char* response);
    bool readCommand();
    
    bool commandQueued;
    char inputCommandBuffer[32];
    
private:
    bool execConfigCommand(const char* command);
    bool execTransmitCommand(const char* command, char* response);
    bool execDiagnosticCommand(const char *command, char* response);
    
    bool commandOverflow;
    size_t inputCommandLen;
};



#endif
