/*
 * Class to handle internal communications in the machine via RS485
 *
 * Adrian Bowyer 3 July 2009
 *
 */
 
 /* 
 
   A data packet looks like this:
 
     * * T F S A d a t a $
     
         0 1 2 3 4 . . .
     
   All these are printable characters.  * is the start character; it is not included in the string that is the result of 
   reading the packet.  There may be more than one to allow the comms to stabilise.  T (to) is the one-char name of the destination, 
   F (from) is the one-char name of the source, d a t a is a char string containing the message.  The checksum is S; this is
   calculated by adding all the bytes of the message (T F A and data, but not itself nor the start and end characters), taking the last four bits
   of the count, and adding that to the '0' character.  A is either RS485_ACK or RS485_ERROR.  The packet is terminated by a single $ character.    The total length of 
   a packet with all of that should not exceed RS485_BUF_LEN (defined in configuration.h) characters.  When a packet is received 
   the $ character is replaced by 0, thus forming a standard C string.
   
   Error returns: bool functions return true for success, false for failure.
 
 */
 
#ifndef INTERCOM_H
#define INTERCOM_H

#endif
