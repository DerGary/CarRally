/****************************************************************************/
/* Supported Microcontroller :RX62T                                         */
/* File contents  :printf,scanfrelated processing                           */
/* Version        :Ver.1.00                                                 */
/* Date           :2013.09.04                                               */
/* Copyright      :Renesas Micom Car Rally Secretariat                      */
/****************************************************************************/

/*======================================*/
/* Include                              */
/*======================================*/
#include    <stdio.h>                   //
#include    <machine.h>
#include    "iodefine.h"
#include    "printf_lib.h"              // printf related processing

/*======================================*/
/* Symbol definitions                   */
/*======================================*/
#define     SEND_BUFF_SIZE  64          // send buffer size
#define     RECV_BUFF_SIZE  32          // receive buffer size

/*======================================*/
/* Global variable declarations         */
/*======================================*/
/* send buffer */
static volatile char    send_buff[SEND_BUFF_SIZE];
static volatile char    *send_w = send_buff;
static volatile char    *send_r = send_buff;
static volatile int     send_count = 0;

/* receive buffer */
static volatile char    recv_buff[RECV_BUFF_SIZE];
static volatile char    *recv_w = recv_buff;
static volatile char    *recv_r = recv_buff;

static int             recvFlag;        // receive flag
static unsigned char   recvData;        // receive data
static unsigned char   recvError;       // receive error

/*======================================*/
/* Prototype declarations               */
/*======================================*/
void setSendBuff( char c);
int getSendBuff( char *c );

/************************************************************************/
/* UART0 Initialization printf assign relation to UARTO                 */
/* Argument�@ transmission speed                                        */
/* Return values: none                                                  */
/************************************************************************/
void init_sci1_printf( int sp )
{
    int i;

    MSTP(SCI1) = 0 ;                    // Wakeup SCI1

    PORTD.ICR.BIT.B5 = 1;               // RxD1input buffer ON

    SCI1.SCR.BYTE = 0 ;                 // select internal clock

    // BRR = PCLK * 10^6 / ( 64 * 2^(2n-1) * B) - 1
    // PCLK = 12.288*4, n=depend on PCLK clock B=baud rate[bps]
    // n = PCLK/1:0 PCLK/4:1 PCLK/16:2 PCLK/64:3
    if( sp == SPEED_4800 ) {
        SCI1.SMR.BYTE = 0x01 ;          // PLCK/4, asynchronous,bit 8, Parity none
        SCI1.BRR = 80-1;                // 49152000 / ( 64 * 2 * 4800 ) - 1
    } else if( sp == SPEED_9600 ) {
        SCI1.SMR.BYTE = 0x00 ;          // PLCK, asynchronous,bit 8, Parity none
        SCI1.BRR = 160-1;               // 49152000 / ( 64 * 0.5 * 9600 ) - 1
    } else if( sp == SPEED_19200 ) {
        SCI1.SMR.BYTE = 0x00 ;          // PLCK, asynchronous,bit 8, Parity none
        SCI1.BRR = 80-1;                // 49152000 / ( 64 * 0.5 * 19200 ) - 1
    } else if( sp == SPEED_38400 ) {
        SCI1.SMR.BYTE = 0x00 ;          // PLCK, asynchronous,bit 8, Parity none
        SCI1.BRR = 40-1;                // 49152000 / ( 64 * 0.5 * 38400 ) - 1
    }
    for(i=0; i<4000; i++);              // bit 1 term(52��s,ICLK=96MHz)wait
    SCI1.SCR.BYTE = 0xf0;               // Transmission and reception permission,
                                        // Transmission and reception interrupt permission

    IPR(SCI1,    ) = 7 ;                // SCI1 interrupt prior level=7
    IEN(SCI1,TXI1) = 1 ;                // SCI1 receive interrupt permission(IEN)
    IEN(SCI1,RXI1) = 1 ;                // SCI1 receive interrupt permission(IEN)
    IEN(SCI1,ERI1) = 1 ;                // SCI1 receive error interrupt permission(IEN)
}

/************************************************************************/
/* Called from a printf function                                        */
/* Can not call it from user                                            */
/************************************************************************/
long write(long fileno, const unsigned char *buf, long count)
{
    char   put_data;

    if( *buf == '\n' )  {
        setSendBuff( '\r' );
    } else if( *buf == '\b' ) {
        setSendBuff( '\b' );
        setSendBuff( ' ' );
    }

    setSendBuff( *buf );

    // �������M���Ă��Ȃ��Ȃ�A1�����ڂ͂����ő��M����
    // ���̌�͑��M���荞�݂ő��M����
    if( SCI1.SSR.BIT.TEND == 1 ) {
        getSendBuff( &put_data );
        put_sci1( put_data );
    }

    return 1;
}

/************************************************************************/
/* Called from a scanf function                                         */
/* Can not call it from user                                            */
/************************************************************************/
long read(long fileno, unsigned char *buf, long count)
{
    char   c;

    if( recv_r == recv_w ) {
        do {
            /* Wait for receive */
            while( get_sci1( &c ) != 1 );

            switch( c ) {
            case '\b':  /* Back space */
                /* �����o�b�t�@�ɂȂ��Ȃ�BS�͖��� */
                if( recv_r == recv_w ) continue;
                /* ����Ȃ��߂� */
                recv_w--;
                break;
            case '\r':  /* Enter key */
                *recv_w++ = c = '\n';
                *recv_w++ = '\r';
                break;
            default:
                if( recv_w >= recv_buff+RECV_BUFF_SIZE-2 ) continue;
                *recv_w++ = c;
                break;
            }
            /* �G�R�[�o�b�N ���͂��ꂽ������Ԃ� */
			write( 0, &(unsigned char)c, 1 );
        } while( c != '\n' );
    }

    *buf = *recv_r++;
    if( recv_r == recv_w ) recv_r = recv_w = recv_buff;

    return 1;
}

/************************************************************************/
/* Receive for 1 character                                              */
/* Arguments:�@   Address of received character                         */
/* Return values: -1:Receive error 0:Receive none 1:Receive             */
/************************************************************************/
int get_sci1( char *s )
{
    volatile int ret = 0;

    if( recvFlag == 1 ){                // ��M�f�[�^����H
        recvFlag = 0;
        if (s) {
        	*s = recvData;
        }
        ret = 1;
        if( recvError ) {               // �G���[����H
            recvError = 0;
            ret = -1;
        }
    }
    return ret;
}

/************************************************************************/
/* Send for 1 character                                                 */
/* Arguments:�@   Send data                                             */
/* Return values: 0:For sending 1:Set of send data                      */
/************************************************************************/
int put_sci1( char r )
{
    if( SCI1.SSR.BIT.TDRE == 1) {       // TDR�ɏ������݂��o����܂ő҂�
        SCI1.TDR = r;
        return 1;
    } else {
        /* ���M��(����̃f�[�^�͑��M�����ɏI��) */
        return 0;
    }
}

/************************************************************************/
/* Saving the send buffer                                               */
/* Arguments:�@   Character data                                        */
/* Return values: None                                                  */
/* ����   �o�b�t�@���t���̏ꍇ�A�󂭂܂ő҂��܂�                        */
/************************************************************************/
void setSendBuff( char c )
{
    // �o�b�t�@���󂭂܂ő҂�
    while( SEND_BUFF_SIZE == send_count );

    IEN(SCI1,TXI1) = 0;

    *send_w++ = c;
    if( send_w >= send_buff+SEND_BUFF_SIZE ) send_w = send_buff;
    send_count++;

    IEN(SCI1,TXI1) = 1;
}

/************************************************************************/
/* Acquisition the send buffer                                          */
/* Arguments:     Adress of character                                   */
/* Return values: 0:Data none 1:data                                    */
/************************************************************************/
int getSendBuff( char *c )
{
    volatile int    ret = 0;

    if( send_count ) {                  // �f�[�^������Ȃ�o�b�t�@����o��
        IEN(SCI1,TXI1) = 0;

        *c = *send_r++;
        if( send_r >= send_buff+SEND_BUFF_SIZE ) send_r = send_buff;
        send_count--;
        ret = 1;

        IEN(SCI1,TXI1) = 1;
    }
    return ret;
}

/************************************************************************/
/* SCI1 ERI1 Interrupt                                                  */
/************************************************************************/
#pragma interrupt Excep_SCI1_ERI1(vect=VECT_SCI1_ERI1)
void Excep_SCI1_ERI1(void)
{
    recvError = SCI1.SSR.BYTE & 0x38;   // ��M�G���[�t���O�ǂݏo��

    SCI1.SSR.BYTE = 0xc0 ;              // ��M�G���[�t���O�N���A
    while( (SCI1.SSR.BYTE & 0x38) );    // �G���[�t���O�́g0�h�N���A�m�F
    while( IR(SCI1,ERI1) );             // ��M�G���[�̊��荞�݃X�e�[�^�X�r�b�g��0���m�F
}

/************************************************************************/
/* SCI1 RXI1 Interrupt                                                  */
/************************************************************************/
#pragma interrupt Excep_SCI1_RXI1(vect=VECT_SCI1_RXI1)
void Excep_SCI1_RXI1(void)
{
    recvData = SCI1.RDR ;               // ��M�f�[�^�ǂݏo��
    recvFlag = 1 ;                      // �t���O�ϐ����P�ɃZ�b�g
}

/************************************************************************/
/* SCI1 TXI1 Interrupt                                                  */
/************************************************************************/
#pragma interrupt Excep_SCI1_TXI1(vect=VECT_SCI1_TXI1)
void Excep_SCI1_TXI1(void)
{
    char   c;
    int    ret;

    ret = getSendBuff( &c );            // �f�[�^�擾
    if( ret ) {
        SCI1.TDR = c;                   // �f�[�^����Ȃ瑗�M
    }
}

/************************************************************************/
/* end of file                                                          */
/************************************************************************/

