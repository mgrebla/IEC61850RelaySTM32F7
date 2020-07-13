#include "libiec61850_platform_includes.h"
#include "hal_ethernet.h"
#include "stm32f7xx_hal.h"

struct sEthernetSocket {
    uint32_t transmitBufferAddress;
};

void
Ethernet_getInterfaceMACAddress(const char* interfaceId, uint8_t* addr)
{
		addr[0] = 0x00;
		addr[1] = 0x80;
		addr[2] = 0xE1;
		addr[3] = 0x00;
		addr[4] = 0x00;
		addr[5] = 0x00;
}

EthernetSocket
Ethernet_createSocket(const char* interfaceId, uint8_t* destAddress)
{

}
/*
void
Ethernet_destroySocket(EthernetSocket ethSocket)
{

}
*/
void
Ethernet_sendPacket(ETH_HandleTypeDef heth, uint8_t* buffer, int packetSize)
{
	memcpy(heth.TxDesc->Buffer1Addr, buffer, packetSize);
	HAL_ETH_TransmitFrame(&heth, packetSize);
}

