#ifndef CORRECTION_RECEIVER_ROS_H
#define CORRECTION_RECEIVER_ROS_H

#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

#include "correction_receiver_ros/serial.hpp"

namespace correction_receiver
{
    
    #define HASH_SIZE (256/8)
    
    struct Config
    {
        std::string rtkTopic;
        std::string rtkFrameID;
        
        std::string broadcastServerIP;
        int broadcastServerPort;
        int receiverLocalPort;
        int authenticationKey;
        int timeoutReAuth;
        int maxTransferredDataSize;
        
        std::string rtkServiceSerial;
        int serialBaudRate;
        int serialDataBits;
        int serialStopBits;
        int serialParity;
    };
    
    typedef uint8_t HashValue[HASH_SIZE];

    class CorrectionReceiver
    {
    public:
        CorrectionReceiver(Config & conf, ros::NodeHandle & nh_);
        ~CorrectionReceiver();
        
        void run();
        
    private:
        Config config;
        
        ros::NodeHandle nh;
        ros::Publisher rtkPub;
        
        uint8_t *buffer;
        
        boost::asio::io_service ioService;
        boost::asio::ip::udp::socket socket;
        boost::asio::ip::udp::endpoint serverEndPoint;
        
        serial sx;
        
        sensor_msgs::NavSatFix cvt2NavSatFix(double latitude, double longitude, double height, 
                                            int Q, int ns, double sdn, double sde, double sdu, 
                                            double sdne, double sdeu, double sdun);
        void llhparser(const char *llh, int bytes);
        
        bool initialized;
        long curPlainID;
        HashValue curHashVal;
        uint8_t *shaMsgBuffer;
        int bytesToProcess;
        
        int timeoutCount;
    };

}


namespace tiny_sha3
{
    
    #ifndef ROTL64
    #define ROTL64(x, y) (((x) << (y)) | ((x) >> (64 - (y))))
    #endif
    
    // state context
    struct sha3_ctx_t
    {
        union// state:
        {
            uint8_t b[200];                     // 8-bit bytes
            uint64_t q[25];                     // 64-bit words
        } st;
        
        int pt, rsiz, mdlen;                    // these don't overflow
    };

    // Compression function.
    void sha3_keccakf(uint64_t st[25]);

    // OpenSSL - like interfece
    int sha3_init(sha3_ctx_t *c, int mdlen);    // mdlen = hash output in bytes
    int sha3_update(sha3_ctx_t *c, const void *data, size_t len);
    int sha3_final(void *md, sha3_ctx_t *c);    // digest goes to md
    
    // compute a sha3 hash (md) of given byte length from "in"
    void *sha3(const void *in, size_t inlen, void *md, int mdlen);
    
    void shake_xof(sha3_ctx_t *c);
    void shake_out(sha3_ctx_t *c, void *out, size_t len);

}

#endif