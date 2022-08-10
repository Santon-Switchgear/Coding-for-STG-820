# Coding project
Project coding for STG-826 PLC
1 analog input
5 digital Inputs
1 analog Output
3 digital Outputs
CANopen Communication Protocol with CANopenNode needs to work



I am looking for an example of code where this communication functions with the initialization and pre-operational configuration and with trigger from client will move to Operational.

I need to finish this as fast as possible. If there is someone that belives they can help me do this I will be willing to compansate them.
The initial deadline i was given and will not reach was 5 september.
(PS. On holiday and unavailable from 12th till the 28th of august 2022)

I have got it to transmit and gather the application data andcan send a contant stream of the data. I understance CanOpen i cannot find out how to put it into the code using the existing CanOpen Code.
How can I use the Canopen Library itself with HAL to run a standard CAN Open Communication with PDO and SDO implementation.
I am looking for an example of code where this communication functions with the initialization and pre-operational configuration and with trigger from client will move to Operational.

The states i go through:
![CanOpen_Device_State_transitions](https://user-images.githubusercontent.com/66464852/183848657-9a4edd96-82f4-4479-b803-5ccd5e11f521.png)
1.	0x00 Initialization --> Pre-Operational: TX one message 700+NodeID   with 0x00 (Status = Initialization)
SDO communication not possible
PDO communication not possible
2.	0x7F Pre-Operational: TX message 700+NodeID   with 0x7F (Status = Pre-Operational)
SDO communication possible
PDO communication not possible
3.	0x05 Operational: TX message 700+NodeID   with 0x05 (Status = Operational)
SDO communication possible
PDO communication possible
4.	0x04 Stop: TX message 700+NodeID   with 0x04 (Status = Stop)
SDO communication not possible
PDO communication not possible
NMT messages come only from master and control the client states. It can be for all (Identifier=0x000, DLC=1, command at Byte 0) or for one device (Identifier=0x000, DLC>=2, command at Byte 0, nodeID at Byte 1)

Byte 0:
1.	0x01 Start Remote Node 
2.	0x02 Stop Remote Node 
3.	0x80 Enter Pre-Operational 
4.	0x81 Reset Node
5.	0x82 Reset Communication


The problem is that i need to first use Canopen for initialization.
 TPDO + RPDO:
![TPDO1 +RPDO1 table](https://user-images.githubusercontent.com/66464852/183849206-52b9fd15-5a8f-4069-b8a9-00663667b6a0.png)

I Currently am using the main program located in:
Coding-for-STG-820\PLC1\Src\main.c
Source code:
[https://github.com/Santon-Switchgear/Coding-for-STG-820.git](url)

![CanOpen_Device_Communicatio_protocol](https://user-images.githubusercontent.com/66464852/183848681-0d6ad55c-8ed6-4990-9346-47f0207dfe04.png)

