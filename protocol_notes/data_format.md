### Data format
+ packet data

| SIZE  | PID | TN  | SN  | FT  | PAYLOAD |
| ----- | --- | --- | --- | --- | ------- |
| Bytes | 1   | 2   | 2   | 1   | N       |

### Local bitmap
+ \#define KBYTES_SUPPORT_MAX  (128)
+ 128*8 bits in total, each represent a 128 Bytes data
+ record 128KB file at most
+ bitmap[KBYTES_SUPPORT_MAX] in local

### Data allignment
+ PAYLOAD = MTU - OPCODE - PID - TN - SN - FT
+ P_ALLIGNE_SIZE = PAYLOAD - 1
+ value of TN = (file size + P_ALLIGNE_SIZE) / PAYLOAD, which represent the max number of PAYLOAD
+ SN initialize to 0x0000

### Local PID ---
+ what is this var?
+ new packet arrives, updates local PID
+ SN is idnetified with each specific bit

| BIT | 1   | 2   | .   | 10  | .   | .   | .   | 128 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| SN  | 1   | 2   |     |     |     |     |     | 128 |
