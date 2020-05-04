/*
   https://github.com/mgk/arduino-xmodem.git
*/

typedef enum {
  Crc,
  ChkSum
} transfer_t;


class XModem {
  private:
    //delay when receive bytes in frame - 7 secs
    static const int receiveDelay;
    //retry limit when receiving
    static const int rcvRetryLimit;
    //holds readed byte (due to dataAvail())
    int byte;
    //expected block number
    unsigned char blockNo;
    //extended block number, send to dataHandler()
    unsigned long blockNoExt;
    //retry counter for NACK
    int retries;
    //buffer
    char buffer[128];
    //repeated block flag
    bool repeatedBlock;

    int  (*recvChar)(int);
    void (*sendChar)(char);
    bool (*dataHandler)(unsigned long number, char *buffer, int len);
    unsigned short crc16_ccitt(char *buf, int size);
    bool dataAvail(int delay);
    int dataRead(int delay);
    void dataWrite(char symbol);
    bool receiveFrameNo(void);
    bool receiveData(void);
    bool checkCrc(void);
    bool checkChkSum(void);
    bool receiveFrames(transfer_t transfer, bool filename);
    bool sendNack(void);
    void init(void);

    bool transmitFrames(transfer_t, bool filename);
    unsigned char generateChkSum(void);

  public:
    static const unsigned char NACK;
    static const unsigned char ACK;
    static const unsigned char SOH;
    static const unsigned char EOT;
    static const unsigned char CAN;

    XModem(int (*recvChar)(int), void (*sendChar)(char));
    XModem(int (*recvChar)(int), void (*sendChar)(char),
           bool (*dataHandler)(unsigned long, char*, int));
    bool receive(bool filename = false);
    bool transmit(bool filename = false);
};
