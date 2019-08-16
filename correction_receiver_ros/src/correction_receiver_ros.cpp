#include <correction_receiver_ros/correction_receiver_ros.h>

using namespace std;
using namespace ros;
using boost::asio::ip::udp;

namespace correction_receiver
{
        
    CorrectionReceiver::CorrectionReceiver(Config& conf, NodeHandle& nh_)
    :config(conf), nh(nh_),
    ioService(), socket(ioService, udp::endpoint(udp::v4(), config.receiverLocalPort)), 
    initialized(false), timeoutCount(0)
    {
        rtkPub = nh.advertise<sensor_msgs::NavSatFix>(config.rtkTopic, 1);
        
        buffer = new uint8_t[config.maxTransferredDataSize];
        
        udp::resolver resolver(ioService);
        std::stringstream ss;
        ss << config.broadcastServerPort;    
        udp::resolver::query query(udp::v4(), config.broadcastServerIP, ss.str());
        serverEndPoint = *resolver.resolve(query);
        
        SerialInit(&sx,config.rtkServiceSerial.c_str(), config.serialBaudRate,
                config.serialDataBits, config.serialStopBits, config.serialParity);
        
        shaMsgBuffer = new uint8_t[sizeof(long) + sizeof(int)];
        
    }


    CorrectionReceiver::~CorrectionReceiver()
    {
        SerialFree(&sx);
        if(buffer)
        {
            delete[] buffer;
            buffer = nullptr;
        }
        
        if(shaMsgBuffer)
        {
            delete[] shaMsgBuffer;
            shaMsgBuffer = nullptr;
        }
    }

    void CorrectionReceiver::run()
    {
        int loopRateFreq = 20;
        ros::Rate lr(loopRateFreq);

        while(ros::ok())
        {
            lr.sleep();
            
            if(!initialized)
            {
                char request[] = "request";
                memcpy(buffer, request, sizeof(request));
                socket.send_to(boost::asio::buffer(buffer, sizeof(request)), serverEndPoint);
                
                int bytesRecv = 0;
                if(socket.available() > 0)
                    bytesRecv = socket.receive_from(boost::asio::buffer(buffer, config.maxTransferredDataSize), serverEndPoint);
                if(bytesRecv != sizeof(long))
                    continue;
                else
                {
                    memcpy(&curPlainID, buffer, sizeof(long));
                    memcpy(shaMsgBuffer, &curPlainID, sizeof(long));
                    memcpy(shaMsgBuffer + sizeof(long), &(config.authenticationKey), sizeof(int));
                    tiny_sha3::sha3(shaMsgBuffer, sizeof(long) + sizeof(int), curHashVal, sizeof(HashValue));
                }
                
                initialized = true;
            }
            
            if(initialized)
            {
                memcpy(buffer, &curPlainID, sizeof(long));
                memcpy(buffer + sizeof(long), curHashVal, sizeof(HashValue));
                socket.send_to(boost::asio::buffer(buffer, sizeof(long)+sizeof(HashValue)), serverEndPoint);
                
                bytesToProcess = 0;
                if(socket.available() > 0)
                {
                    timeoutCount = 0;
                    bytesToProcess = socket.receive_from(boost::asio::buffer(buffer, config.maxTransferredDataSize), serverEndPoint);
                }
                else
                {
                    timeoutCount++;
                    if(timeoutCount > loopRateFreq*config.timeoutReAuth)
                    {
                        timeoutCount = 0;
                        initialized = false;
                    }
                }
                
                int ofs = 0;
                while(bytesToProcess > ofs)
                {
                    int i = SerialWrite(&sx, (char*)(buffer+ofs), bytesToProcess-ofs);
                    if(i < 0)
                    {
                        std::cerr << "Could not access serial device" << std::endl;
                        return;
                    }
                    else
                        ofs += i;
                }
                
                int doloop = 1;
                while(doloop)
                {
                    int i = SerialRead(&sx, (char*)buffer, config.maxTransferredDataSize);
                    if(i < 0)
                    {
                        std::cerr << "Could not access serial device" << std::endl;
                        return;
                    }
                    else if (i > 0)
                    {
                        if(i < config.maxTransferredDataSize) doloop = 0;
                        llhparser((char*)buffer, i);
                    }
                    else
                    {
                        doloop = 0;
                    }
                }
            }
            
            ros::spinOnce();
        }
    }

    void CorrectionReceiver::llhparser(const char* llh, int bytes)
    {
        if(bytes > 0)
        {
            double latitude, longitude, height, Q, ns, sdn, sde, sdu, sdne, sdeu, sdun;
            char *data = new char[bytes];
            memcpy(data, llh, bytes);
            char * pch;
            pch = strtok (data," \n\r\t");
            int idx = 0;
            bool findhead = false;
            bool completed = false;
            
            while (pch != NULL)
            {
                if(strstr(pch, "/") != NULL)
                    findhead = true;
                
                switch(idx)
                {
                    case 2:
                        latitude = atof(pch);
                        break;
                    case 3:
                        longitude = atof(pch);
                        break;
                    case 4:
                        height = atof(pch);
                        break;
                    case 5:
                        Q = atof(pch);
                        break;
                    case 6:
                        ns = atof(pch);
                        break;
                    case 7:
                        sdn = atof(pch);
                        break;
                    case 8:
                        sde = atof(pch);
                        break;
                    case 9:
                        sdu = atof(pch);
                        break;
                    case 10:
                        sdne = atof(pch);
                        break;
                    case 11:
                        sdeu = atof(pch);
                        break;
                    case 12:
                        sdun = atof(pch);
                        completed = true;
                        break;
                }
                if(completed) break;
                if(findhead) idx++;
                pch = strtok (NULL, " \n\r\t");
            }
            
            if(completed)
            {
                rtkPub.publish(cvt2NavSatFix(latitude, longitude, height, Q, ns, 
                                            sdn, sde, sdu, sdne, sdeu, sdun));
            }
            
            delete[] data;
            
        }
    }

    sensor_msgs::NavSatFix CorrectionReceiver::cvt2NavSatFix(double latitude, double longitude, double height,
                                                    int Q, int ns, double sdn, double sde, double sdu,
                                                    double sdne, double sdeu, double sdun)
    {
        sensor_msgs::NavSatFix nsf;
        nsf.header.frame_id = config.rtkFrameID;
        nsf.header.stamp = ros::Time::now();
        nsf.latitude = latitude;
        nsf.longitude = longitude;
        nsf.altitude = height;
        nsf.position_covariance.at(0) = sde*sde;
        nsf.position_covariance.at(4) = sdn*sdn;
        nsf.position_covariance.at(8) = sdu*sdu;
        nsf.position_covariance.at(1) = nsf.position_covariance.at(3) = sdne*sdne;
        nsf.position_covariance.at(2) = nsf.position_covariance.at(6) = sdeu*sdeu;
        nsf.position_covariance.at(5) = nsf.position_covariance.at(7) = sdun*sdun;
        switch(Q)
        {
            case 1: nsf.status.status = 1; break;
            case 2: nsf.status.status = 0; break;
            case 3:
            case 4:
            case 5: nsf.status.status = -1; break;
        }
        
        return nsf;
    }

}


namespace tiny_sha3
{
    void sha3_keccakf(uint64_t st[25])
    {
        // constants
        const uint64_t keccakf_rndc[24] = {
            0x0000000000000001, 0x0000000000008082, 0x800000000000808a,
            0x8000000080008000, 0x000000000000808b, 0x0000000080000001,
            0x8000000080008081, 0x8000000000008009, 0x000000000000008a,
            0x0000000000000088, 0x0000000080008009, 0x000000008000000a,
            0x000000008000808b, 0x800000000000008b, 0x8000000000008089,
            0x8000000000008003, 0x8000000000008002, 0x8000000000000080,
            0x000000000000800a, 0x800000008000000a, 0x8000000080008081,
            0x8000000000008080, 0x0000000080000001, 0x8000000080008008
        };
        const int keccakf_rotc[24] = {
            1,  3,  6,  10, 15, 21, 28, 36, 45, 55, 2,  14,
            27, 41, 56, 8,  25, 43, 62, 18, 39, 61, 20, 44
        };
        const int keccakf_piln[24] = {
            10, 7,  11, 17, 18, 3, 5,  16, 8,  21, 24, 4,
            15, 23, 19, 13, 12, 2, 20, 14, 22, 9,  6,  1
        };

        // variables
        int i, j, r;
        uint64_t t, bc[5];

        // actual iteration
        int keccakf_rounds = 24;
        for (r = 0; r < keccakf_rounds; r++) {

            // Theta
            for (i = 0; i < 5; i++)
                bc[i] = st[i] ^ st[i + 5] ^ st[i + 10] ^ st[i + 15] ^ st[i + 20];

            for (i = 0; i < 5; i++) {
                t = bc[(i + 4) % 5] ^ ROTL64(bc[(i + 1) % 5], 1);
                for (j = 0; j < 25; j += 5)
                    st[j + i] ^= t;
            }

            // Rho Pi
            t = st[1];
            for (i = 0; i < 24; i++) {
                j = keccakf_piln[i];
                bc[0] = st[j];
                st[j] = ROTL64(t, keccakf_rotc[i]);
                t = bc[0];
            }

            //  Chi
            for (j = 0; j < 25; j += 5) {
                for (i = 0; i < 5; i++)
                    bc[i] = st[j + i];
                for (i = 0; i < 5; i++)
                    st[j + i] ^= (~bc[(i + 1) % 5]) & bc[(i + 2) % 5];
            }

            //  Iota
            st[0] ^= keccakf_rndc[r];
        }
    }

    // Initialize the context for SHA3

    int sha3_init(sha3_ctx_t *c, int mdlen)
    {
        int i;

        for (i = 0; i < 25; i++)
            c->st.q[i] = 0;
        c->mdlen = mdlen;
        c->rsiz = 200 - 2 * mdlen;
        c->pt = 0;

        return 1;
    }

    // update state with more data

    int sha3_update(sha3_ctx_t *c, const void *data, size_t len)
    {
        size_t i;
        int j;

        j = c->pt;
        for (i = 0; i < len; i++) {
            c->st.b[j++] ^= ((const uint8_t *) data)[i];
            if (j >= c->rsiz) {
                sha3_keccakf(c->st.q);
                j = 0;
            }
        }
        c->pt = j;

        return 1;
    }

    // finalize and output a hash

    int sha3_final(void *md, sha3_ctx_t *c)
    {
        int i;

        c->st.b[c->pt] ^= 0x06;
        c->st.b[c->rsiz - 1] ^= 0x80;
        sha3_keccakf(c->st.q);

        for (i = 0; i < c->mdlen; i++) {
            ((uint8_t *) md)[i] = c->st.b[i];
        }

        return 1;
    }

    // compute a SHA-3 hash (md) of given byte length from "in"

    void *sha3(const void *in, size_t inlen, void *md, int mdlen)
    {
        sha3_ctx_t sha3;

        sha3_init(&sha3, mdlen);
        sha3_update(&sha3, in, inlen);
        sha3_final(md, &sha3);

        return md;
    }

    // SHAKE128 and SHAKE256 extensible-output functionality

    void shake_xof(sha3_ctx_t *c)
    {
        c->st.b[c->pt] ^= 0x1F;
        c->st.b[c->rsiz - 1] ^= 0x80;
        sha3_keccakf(c->st.q);
        c->pt = 0;
    }

    void shake_out(sha3_ctx_t *c, void *out, size_t len)
    {
        size_t i;
        int j;

        j = c->pt;
        for (i = 0; i < len; i++) {
            if (j >= c->rsiz) {
                sha3_keccakf(c->st.q);
                j = 0;
            }
            ((uint8_t *) out)[i] = c->st.b[j++];
        }
        c->pt = j;
    }

}
