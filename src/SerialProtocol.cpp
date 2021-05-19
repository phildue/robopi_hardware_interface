//
// Created by phil on 18.05.21.
//

#include <algorithm>
#include "SerialProtocol.h"

SerialProtocol::MsgType SerialProtocol::Message::msgType(const std::string &str) {
    if ( str == "S")
    {
        return MsgType::STATE;
    }
    if ( str == "v" )
    {
        return MsgType::CMD_VEL;

    }
    return MsgType::UNKNOWN;
}

std::string SerialProtocol::Message::to_string(SerialProtocol::MsgType msgType) {
    switch(msgType)
    {
        case MsgType ::CMD_VEL:
            return "v";
        case MsgType ::STATE:
            return "S";
        default:
            return "";
    }
}

SerialProtocol::Message::Message(const std::string &msg):_str(msg) {
    _fields = split(msg,' ');
    _type = msgType(_fields[0]);
    _t = std::stoull(_fields[1]);
}

SerialProtocol::Message::Message(SerialProtocol::MsgType type, uint64_t t):
        _t(t),
        _type(type){
    std::stringstream ss;
    ss << to_string(_type) << " " << t << "\n";
    _str = ss.str();

}


SerialProtocol::MsgCmdVel::MsgCmdVel(const std::string &raw):Message(raw) {
    _vl = std::stod(_fields[2]);
    _vr = std::stod(_fields[3]);
}
SerialProtocol::MsgCmdVel::MsgCmdVel(float vl, float vr, uint64_t t) :Message(MsgType::CMD_VEL,t),_vl(vl),_vr(vr)
{
    std::stringstream ss;
    ss << to_string(_type) << " " << t << " " << vl << " " << vr << "\n";
    _str = ss.str();
}

SerialProtocol::MsgState::MsgState(const std::string& raw):Message(raw)
{
    _stateLeft.position = std::stof(_fields[2]);
    _stateLeft.angularVelocityCmd = std::stof(_fields[3]);
    _stateLeft.angularVelocity = std::stof(_fields[4]);
    _stateLeft.err = std::stof(_fields[5]);
    _stateLeft.dutySet = std::stof(_fields[6]);

    _stateRight.position = std::stof(_fields[7]);
    _stateRight.angularVelocityCmd = std::stof(_fields[8]);
    _stateRight.angularVelocity = std::stof(_fields[9]);
    _stateRight.err = std::stof(_fields[10]);
    _stateRight.dutySet = std::stof(_fields[11]);

}
SerialProtocol::MsgState::MsgState(const State& stateLeft, const State& stateRight, uint64_t t):_stateLeft(stateLeft),_stateRight(stateRight),Message(MsgType::STATE,t)
{
    std::stringstream ss;
    ss << to_string(_type) << " " << t << " " << stateLeft.position << " " << stateLeft.angularVelocityCmd << " " << stateLeft.angularVelocity << " " << stateLeft.err;
    ss << stateRight.position << " " << stateRight.angularVelocityCmd << " " << stateRight.angularVelocity << " " << stateRight.err;
    ss << "\n";
    _str = ss.str();
}
std::string SerialProtocol::MsgState::str() const {
    std::stringstream ss;
    ss.precision(4);
    ss << "       |" << std::setw(6) <<"L" << " |" << std::setw(8) <<"R|\n";
    ss << " t |" << std::setw(6) << (double)_t << " |" << std::setw(6) << (double)_t << "|\n";
    ss << " p |" << std::setw(6) << _stateLeft.position << " |" << std::setw(6) << _stateRight.position << "|\n";
    ss << " v*    |" << std::setw(6) << _stateLeft.angularVelocityCmd << " |" << std::setw(6) << _stateRight.angularVelocityCmd << "|\n";
    ss << " v     |" << std::setw(6) << _stateLeft.angularVelocity << " |" << std::setw(6) << _stateRight.angularVelocity << "|\n";
    ss << " err   |" << std::setw(6) << _stateLeft.err << " |" << std::setw(6) << _stateRight.err << "|\n";
    ss << " pwm   |" << std::setw(6) << _stateLeft.dutySet << " |" << std::setw(6) << _stateRight.dutySet << "|\n";
    return ss.str();
}

void SerialProtocol::read(int nMessages) {
    int nBytes = 0;
    int readMessages = 0;
    do{
        nBytes = _serialPort.Read(_buffer,1);
        if (_buffer.str().back() == '\n')
        {
            auto raw = _buffer.str();
            try{
                auto msgType = Message::msgType(raw.substr(0,1));
                switch(msgType)
                {
                    case MsgType::STATE:
                        _messagesState.push_back(std::make_shared<MsgState>(raw));
                        break;
                    case MsgType::CMD_VEL:
                        _messagesCmdVel.push_back(std::make_shared<MsgCmdVel>(raw));
                        break;
                    default:
                        _messages.push_back(std::make_shared<Message>(raw));
                }

                _buffer = std::stringstream();
                readMessages++;
            }catch(const std::exception& e)
            {
                _buffer = std::stringstream();
                throw ParseError(e.what(),raw);
            }


        }
    }while(nBytes > 0 && readMessages < nMessages);
}

void SerialProtocol::send(const std::shared_ptr<const SerialProtocol::Message> &msg) {
    _serialPort.Write(msg->serialStr());

}

std::vector<std::string> SerialProtocol::split(const std::string &s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}

void SerialProtocol::clear() {
    _messages.clear();
    _messagesState.clear();
    _messagesCmdVel.clear();
}
