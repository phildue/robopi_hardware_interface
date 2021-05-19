//
// Created by phil on 18.05.21.
//

#ifndef ROBOPI_SERIALPROTOCOL_H
#define ROBOPI_SERIALPROTOCOL_H


#include <CppLinuxSerial/SerialPort.hpp>
using namespace mn::CppLinuxSerial;

#include <chrono>
#include <thread>
#include <cmath>
#include <string>
#include <iostream>
#include <iomanip>


class SerialProtocol
{
public:

    class ParseError : public std::runtime_error
    {
    public:
        ParseError(const std::string& errorMessage,const std::string& parsedMessage):std::runtime_error(errorMessage),_parsedMessage(parsedMessage)
        {

        }
        const std::string _parsedMessage;
    };

    enum class MsgType{
        UNKNOWN,
        CMD_VEL,
        STATE
    };
    struct Message
    {
    public:
        Message(const std::string& msg);
        Message(MsgType type, uint64_t t);
        const std::string& serialStr() const {return _str;};
        const MsgType& type() const {return _type;};
        const uint64_t& t() const {return _t;}
        virtual std::string str() const { return _str;}

        static MsgType msgType(const std::string& str);
        static std::string to_string(MsgType msgType);
    protected:
        std::string _str;
        MsgType _type;
        uint64_t _t;
        std::vector<std::string> _fields;


    };
    struct MsgCmdVel : public Message
    {
        float _vl,_vr;

        MsgCmdVel(const std::string& raw);
        MsgCmdVel(float vl, float vr, uint64_t t);
    };

    struct State
    {
        float angularVelocityCmd, angularVelocity,position,err,dutySet;

    };
    struct MsgState : public Message
    {
        State _stateLeft, _stateRight;
        MsgState(const std::string& raw);
        MsgState(const State& stateLeft, const State& stateRight, uint64_t t);
        std::string str() const override ;

    };

    std::vector<std::shared_ptr<const Message>> _messages;
    std::vector<std::shared_ptr<const MsgCmdVel>> _messagesCmdVel;
    std::vector<std::shared_ptr<const MsgState>> _messagesState;
    SerialPort _serialPort;

    void read(int nMessages);

    void send(const std::shared_ptr<const Message>& msg);

    static std::vector<std::string> split(const std::string& s, char delimiter);
    void clear();
private:
    std::stringstream _buffer;
};


#endif //SRC_SERIALPROTOCOL_H
