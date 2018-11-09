#ifndef CONFIG_UART_EXTENDED
#define CONFIG_UART_EXTENDED

#define OPERATING_CHANNEL ( 11 ) // !< Channel to transmit on. Must be between 11 and 26!
#define PAN_ID            ( 0xBEAF ) //!< System PAN ID.

//Make sure that the two nodes are programmed with different short address 
//(SHORT_ADDRESS) and destination address (DEST_ADDRESS). Do this by commenting
//out one of the sections below. First compile for Node1 and then for Node2.

//Node1
#define SHORT_ADDRESS_NODE1 ( 0xBAAD ) //!< Short Addres for node 1 of the link.

//Node2
#define SHORT_ADDRESS_NODE2 ( 0xACDC ) //!< Short Addres for node 2 of the link.

/*Compile and program once for each of these options.*/
//#define NODE1
#define NODE2

#ifdef NODE1
#define SHORT_ADDRESS ( SHORT_ADDRESS_NODE1 )
#define DEST_ADDRESS  ( SHORT_ADDRESS_NODE2 )
#else
#define SHORT_ADDRESS ( SHORT_ADDRESS_NODE2 )
#define DEST_ADDRESS  ( SHORT_ADDRESS_NODE1 )
#endif

/*Chose one of these options according to the kit you want to use.*/
//#define STK541
#define RZ502

#define COM_RX_BUFFER_SIZE ( 118 ) //DO NOT ALTER!!!
#define RX_POOL_SIZE       ( 4 ) //MUST BE GREATER THAN ZERO.
#endif
/*EOF*/
